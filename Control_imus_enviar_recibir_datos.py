import time
import math
import serial
import board
import busio
import adafruit_bno055
import numpy as np
from scipy.spatial.transform import Rotation as R
import platform as pl
# ==========================
# Variables globales
# ==========================
torque_M1 = torque_M2 = torque_M3 = 0.0
vel_M1 = vel_M2 = vel_M3 = 0.0

pos_x = pos_y = 0.0
vel_x = vel_y = 0.0
phi = phi_dot = 0.0

alpha = beta = 0.0
alpha_dot = beta_dot = 0.0

prev_angles = None
prev_phi = None

def wrap_to_pi(theta):
    return np.arctan2(np.sin(theta), np.cos(theta))

# ==========================
# Configuración Serial
# ==========================
ser = serial.Serial(
    port='/dev/ttyAMA0',  # Para Raspberry Pi 5 (ajusta si usas /dev/ttyS0, /dev/ttyAMA10, /dev/serial0)
    baudrate=115200,
    timeout=1
)

def recibir_datos():
    global torque_M1, torque_M2, torque_M3, vel_M1, vel_M2, vel_M3
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').strip()
        valores = data.split(",")
        if len(valores) == 6:
            try:
                torque_M1, torque_M2, torque_M3, vel_M1, vel_M2, vel_M3 = map(float, valores)
                print(f"[UART] Recibidos: {torque_M1}, {torque_M2}, {torque_M3}, {vel_M1}, {vel_M2}, {vel_M3}")
            except ValueError:
                print("[UART] Error en conversión de datos")
        else:
            print("[UART] Error: cantidad incorrecta de valores ->", len(valores))

def enviar_datos(u):
    """
    u: array o lista con 3 valores (entradas de control)
    """
    mensaje = f"<{u[0]},{u[1]},{u[2]}>\n"
    ser.write(mensaje.encode('utf-8'))
    print(f"[UART] Enviado: {mensaje.strip()}")
    
def normalize_angle(angle):
    return (angle + 180) % 360 - 180

# ==========================
# Configuración IMUs
# ==========================
i2c = busio.I2C(board.SCL, board.SDA)
imu1 = adafruit_bno055.BNO055_I2C(i2c, address=0x29) 
imu2 = adafruit_bno055.BNO055_I2C(i2c, address=0x28)

time.sleep(1)  # Esperar a que inicialicen correctamente

# Leer la orientación inicial (en grados)
initial_euler_imu1 = imu1.euler
initial_euler_imu2 = imu2.euler

print("Calibrando...")

# Esperar a que ambos IMUs entreguen lecturas válidas
while (initial_euler_imu1 is None or initial_euler_imu2 is None):
    time.sleep(0.1)
    initial_euler_imu1 = imu1.euler
    initial_euler_imu2 = imu2.euler

print("Calibración inicial completa.")
print("IMU1 base:", initial_euler_imu1)
print("IMU2 base:", initial_euler_imu2)

# Función para normalizar ángulos a (-pi, pi)
def wrap_to_pi(angle_rad):
    return (angle_rad + math.pi) % (2 * math.pi) - math.pi
def leer_imus(dt):
    global alpha, beta, alpha_dot, beta_dot, phi, phi_dot, prev_angles, prev_phi

    # ---- IMU 1 ----
    quaternion1 = imu1.quaternion
    if quaternion1 is not None and not np.allclose(quaternion1, (0.0, 0.0, 0.0, 0.0)):
        
        #r = R.from_quat(quaternion1, scalar_first=True)
        q_xyzw = np.array([quaternion1[1], quaternion1[2], quaternion1[3], quaternion1[0]])
        r = R.from_quat(q_xyzw)
        euler_yzx_rad = r.as_euler('yzx', degrees=False)
    else:
        euler_yzx_rad = (0.0, 0.0, 0.0)

    alpha, beta = euler_yzx_rad[0]*np.pi/180, euler_yzx_rad[1]

    if prev_angles is not None:
        alpha_dot = (alpha - prev_angles[0]) / dt
        beta_dot = (beta - prev_angles[1]) / dt
    else:
        alpha_dot = beta_dot = 0.0
    prev_angles = (alpha, beta)

    # ---- IMU 2 ----
    euler2 = imu2.euler
    if euler2 is not None and euler2[0] is not None:
        phi = np.deg2rad(euler2[0])
        phi = wrap_to_pi(phi)
    else:
        phi = 0.0

    if prev_phi is not None:
        phi_dot = (phi - prev_phi) / dt
    else:
        phi_dot = 0.0
    prev_phi = phi

    # Debug
    print("[IMU1] alpha={:.2f}° beta={:.2f}° | alpha_dot={:.2f} rad/s, beta_dot={:.2f} rad/s".format(
        np.rad2deg(alpha), np.rad2deg(beta), alpha_dot, beta_dot))
    #print(f"alpha: {alpha} y beta: {beta} y phi: {phi}")
    print("[IMU2] phi={:.2f}° | phi_dot={:.2f} rad/s".format(np.rad2deg(phi), phi_dot))
    print(f"[POSICION] Posición X: {pos_x}, Posición Y: {pos_y}, Vel x: {vel_x}, Vel y: {vel_y}")
# ==========================
# Controlador en espacio de estados
# ==========================
# Popuesta de valores de K:
K = np.array([
    [-19.2601, -7.2982,   0.0000, -592.0059, 131.2501, -44.8296, -15.8977, 0.0000, -166.8300, 37.1499],
    [-7.2982, -19.2601,   0.0000, -131.2501, 592.0059, -15.8977, -44.8296, 0.0000,  -37.1499, 166.8300],
    [0.0000,    0.0000,   1.9370,    0.0000,   0.0000,   0.0000,   0.0000, 1.7144,  -0.0000,  -0.0000]
])

def calculate_control():
    # State vector [x, y, phi, alpha, beta, x_dot, y_dot, phi_dot, alpha_dot, beta_dot]:
    x = np.array([
        #vel_M1, vel_M2, vel_M3,
        pos_x, pos_y, phi,     alpha,     beta,                      
        vel_x, vel_y, phi_dot, alpha_dot, beta_dot 
    ])

    if K.shape[1] != x.shape[0]:
        print(f"[CONTROL] Error: K tiene {K.shape[1]} columnas y x tiene {x.shape[0]} elementos")
        return np.zeros(3)

    u = -K @ x
    return u


# ==========================
# Bucle Principal
# ==========================
dt = 0.075
# 20 ms
start = time.time()
while True:
    
    elapsed = time.time() - start
    if elapsed >= dt:
        print(f"Dt: {elapsed} ")
        start = time.time()
        # 1. UART
        recibir_datos()
        
        # 2.1. Pose & Velocity
        vel_x, vel_y, phi_dot = pl.DKinematics(vel_M1, vel_M2, vel_M3, phi)
        pos_x = pos_x + dt * vel_x 
        pos_y = pos_y + dt * vel_y
        phi   = phi   + dt * phi_dot

        # 2. IMUs
        leer_imus(dt)
        
        # 3. Control
        u = calculate_control()
        #u = [0.15, 0.150, 0.150]
        
        # 4. Enviar acción de control
        enviar_datos(u)

    # Sincronizar tiempo
    #elapsed = time.time() - start
    #print(f"Dt: {elapsed} ")
    #if elapsed >= dt:
    #    time.sleep(dt - elapsed)
    
    

