import time
import math
import serial
import board
import busio
import adafruit_bno055
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import Akima1DInterpolator
import kinematics_platform as pl

# ==========================
# Variables globales
# ==========================

# Targets
#x_goal = y_goal = 0.0
#v_set = [0.0, 0.0, 0.0]
#wt = [0.0, 0.0, 0.0]

# 
count = 0
ready_goal = 0
end = 0
flag = 0
u = [0, 0, 0]

# Measurements
torque_M1 = torque_M2 = torque_M3 = 0.0
vel_M1 = vel_M2 = vel_M3 = 0.0

pos_x = pos_y = 0.0
vel_x = vel_y = 0.0
phi = phi_dot = 0.0

alpha = beta = theta = 0.0
alpha_dot = beta_dot = 0.0

prev_angles = None
prev_phi = None

# ==========================
# Serial Setting
# ==========================
ser = serial.Serial(
    port='/dev/ttyAMA0',
    baudrate=115200,
    timeout=1
)

def recibir_datos():
    global vel_M1, vel_M2, vel_M3, torque_M1, torque_M2, torque_M3, flag
    
    if ser.in_waiting > 0:
        flag = 1
        try:
            data = ser.readline().decode('utf-8').strip()
            valores = data.split(",")

            if len(valores) != 6:
                print(f"[UART] Paquete inválido ({len(valores)} val): '{data}'")
                return  # ignorar este paquete y esperar otro
            
            vel_M1, vel_M2, vel_M3, torque_M1, torque_M2, torque_M3 = map(float, valores)
            print(f"[MOTORS] M1:{vel_M1:.2f}|{torque_M1:.2f}, M2:{vel_M2:.2f}|{torque_M2:.2f}, M3:{vel_M3:.2f}|{torque_M3:.2f}")
            
           
        except ValueError as e:
            print(f"[UART] Error en conversión: {e}")
            print(f"[UART] Dato problemático: '{data}'")
        except Exception as e:
            print(f"[UART] Error inesperado: {e}")

        
def enviar_datos(u):
    mensaje = f"{u[0]:.2f},{u[1]:.2f},{u[2]:.2f}\n"
    ser.write(mensaje.encode('utf-8'))
    #print(f"[UART] Enviado: {mensaje.strip()}")
    #mensaje = f"{wt[0]:.2f},{wt[1]:.2f},{wt[2]:.2f},{end}\n"
    #ser.write(mensaje.encode('utf-8'))
    #print(f"[UART] Enviado: {mensaje.strip()}")

def wrap_to_pi(theta):
    return np.arctan2(np.sin(theta), np.cos(theta))

# ==========================
# IMUs Setup
# ==========================
i2c = busio.I2C(board.SCL, board.SDA)
imu1 = adafruit_bno055.BNO055_I2C(i2c, address=0x29)  # Cuaterniones
imu2 = adafruit_bno055.BNO055_I2C(i2c, address=0x28)  # Euler

time.sleep(1)

# Get starting measurements
quat1_0 = imu1.quaternion
euler2_0 = imu2.euler

# Esperar lecturas válidas
while quat1_0 is None or euler2_0 is None:
    time.sleep(0.1)
    quat1_0 = imu1.quaternion
    euler2_0 = imu2.euler

# ---- IMU1: convertir el cuaternión inicial a Euler ----
q_xyzw_0 = np.array([quat1_0[1], quat1_0[2], quat1_0[3], quat1_0[0]])
r0 = R.from_quat(q_xyzw_0)
euler1_0 = r0.as_euler('yxz', degrees=False)

print("Calibracion inicial completa.")
print("IMU1 (Euler inicial rad):", np.round(euler1_0, 3))
print("IMU2 (Euler inicial degree):", euler2_0)

# ==========================
# Lectura de IMUs
# ==========================
def leer_imus(dt):
    global alpha, beta, alpha_dot, beta_dot, phi, phi_dot, prev_angles, prev_phi

    # ---- IMU1: usa cuaterniones ----
    quat1 = imu1.quaternion
    if quat1 is not None and not np.allclose(quat1, (0.0, 0.0, 0.0, 0.0)):
        q_xyzw = np.array([quat1[1], quat1[2], quat1[3], quat1[0]])
        r = R.from_quat(q_xyzw)
        euler1 = r.as_euler('yxz', degrees=False) # Original planteado: yzx
        # Restar calibración inicial
        euler1_rel = euler1 - euler1_0
        euler1_rel = np.array([wrap_to_pi(a) for a in euler1_rel])
    else:
        euler1_rel = np.zeros(3)

    alpha, beta, theta = euler1_rel[0], euler1_rel[1], euler1_rel[2]

    if prev_angles is not None:
        alpha_dot = (alpha - prev_angles[0]) / dt
        beta_dot = (beta - prev_angles[1]) / dt
    else:
        alpha_dot = beta_dot = 0.0
    prev_angles = (alpha, beta)

    # ---- IMU2: usa Euler directamente ----
    euler2 = imu2.euler
    if euler2 is not None and euler2[0] is not None:
        euler2_rel = np.radians(np.array(euler2) - np.array(euler2_0))
        phi = wrap_to_pi(euler2_rel[0])
    else:
        phi = 0.0
    if prev_phi is not None:
        phi_dot = (phi - prev_phi) / dt
    else:
        phi_dot = 0.0
    prev_phi = phi

    # Debug
    print(f"[IMU1] alpha={np.degrees(alpha):.2f} beta={np.degrees(beta):.2f} theta = {np.degrees(theta):.2f}| alpha dot={alpha_dot:.3f} beta dot={beta_dot:.3f}")
    print(f"[IMU2] phi={np.degrees(phi):.2f} | phi dot={phi_dot:.3f}")
#    print(f"[POS] X={pos_x:.2f}, Y={pos_y:.2f}, Vx={vel_x:.2f}, Vy={vel_y:.2f}")
    print("-" * 70)

# ====#==========================
# Controlador en espacio de estados
#==========================
#Propuesta de valores de K:

#K = np.array([
#    [-19.2601, -7.2982,   0.0000, -592.0059, 131.2501, -44.8296, -15.8977, 0.0000, -166.8300, 37.1499],
#    [-7.2982, -19.2601,   0.0000, -131.2501, 592.0059, -15.8977, -44.8296, 0.0000,  -37.1499, 166.8300],
#   [0.0000,    0.0000,   1.9370,    0.0000,   0.0000,   0.0000,   0.0000, 1.7144,  -0.0000,  -0.0000]
#    ])

#K = np.array([
#    [-19.4402,  -7.3791,   0.0000, -543.3911,  118.7221,  -43.2775,  -15.4071,   0.0000, -134.1367,   29.3166],
#    [ -7.3791, -19.4402,  -0.0000, -118.7221,  543.3911,  -15.4071,  -43.2775,  -0.0000,  -29.3166,  134.1367],
#    [  0.0000,   0.0000,   1.9526,    0.0000,   -0.0000,   0.0000,    0.0000,   1.7295,   -0.0000,   -0.0000]
#])

K = np.array([
    [-19.4402,  -7.3791,   0.0000, -543.3911,  118.7221,  -43.2775,  -15.4071,   0.0000, -134.1367,   29.3166],
    [ -7.3791, -19.4402,  -0.0000, -118.7221,  543.3911,  -15.4071,  -43.2775,  -0.0000,  -29.3166,  134.1367],
    [  0.0000,   0.0000,   1.9526,    0.0000,   -0.0000,   0.0000,    0.0000,   1.7295,   -0.0000,   -0.0000]
])

#K = np.array([
#    [0.0, 0.0000, 0.0000, -592.0059, 131.2501, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000],
#    [0.0, 0.0000, 0.0000, -131.2501, 592.0059, 0.0000, 0.0000, 0.0000,  0.0000, 0.0000],
#    [0.0, 0.0000, 0.0000, 0.0000,   0.0000,   0.0000,   0.0000, 0.0000,  0.0000,  0.0000]
#    ])

#K = 1.0e4 * np.array([
#    [-0.2627, -0.0000,  0.0000, -1.1963,  0.0000, -0.2579, -0.0000,  0.0000, -0.4213,  0.0000],
#    [ 0.0000, -0.2627,  0.0000,  0.0000,  1.1963,  0.0000, -0.2579,  0.0000,  0.0000,  0.4213],
#    [-0.0000,  0.0000,  0.0011, -0.0000, -0.0000, -0.0000,  0.0000,  0.0005, -0.0000, -0.0000]
#    ])

def calculate_control():
    # State vector [x, y, phi, alpha, beta, x_dot, y_dot, phi_dot, alpha_dot, beta_dot]:
    global K
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

#======================
# Bucle Principal
# ==========================
dt = 0.02
torque = [0.0, 0.0, 0.0]
start = time.time()
while True:
    elapsed = time.time() - start
    #print(elapsed)
    if elapsed >= dt:
        start = time.time()
        
        # 1. UART
        recibir_datos()
        enviar_datos(torque)
        # 2. Read IMUs
        #leer_imus(dt)
        
        if flag == 1:
            flag = 0

            leer_imus(dt)

            # 3. Kinematics
            vel_x, vel_y, phi_dot = pl.DKinematics(vel_M1, vel_M2, vel_M3, phi)
            pos_x += dt * vel_x
            pos_y += dt * vel_y
            #phi   += dt * phi_dot
            
            # 4. Path tracking algorithm
            u = calculate_control()
            torque = pl.transform_velocities(u,phi)
            torque = np.clip(torque, -10.0, 10.0)
            print(f"torque: {torque} ")
                       

           # print(f"Tiempo de procesamiento: {elapsed}")

