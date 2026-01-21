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
path = np.array([[0.0,0.0],
                [0.3,0.0],
                [0.3+0.3*np.sin(np.pi/8),0.3-0.3*np.cos(np.pi/8)],
                [0.3+0.3*np.sin(np.pi/4),0.3-0.3*np.cos(np.pi/4)],
                [0.3+0.3*np.sin(np.pi*3/8),0.3-0.3*np.cos(np.pi*3/8)],
                [0.6,0.3],
                [0.6,0.6],
                [0.6+0.6-0.6*np.cos(np.pi/8),0.6+0.6*np.sin(np.pi/8)],
                [0.6+0.6-0.6*np.cos(np.pi/4),0.6+0.6*np.sin(np.pi/4)],
                [0.6+0.6-0.6*np.cos(np.pi*3/8),0.6+0.6*np.sin(np.pi*3/8)],
                [1.2,1.2],
                [1.6,1.2],
                [1.6 + 0.2*np.sin(np.pi/4),1.2 + 0.2 - 0.2*np.cos(np.pi/4)],
                [1.8,1.4],
                [1.8,2.2],
                [1.8 - 0.4 + 0.4*np.cos(np.pi/8),2.2 + 0.4*np.sin(np.pi/8)],
                [1.8 - 0.4 + 0.4*np.cos(np.pi/4),2.2 + 0.4*np.sin(np.pi/4)],
                [1.8 - 0.4 + 0.4*np.cos(np.pi*3/8),2.2 + 0.4*np.sin(np.pi*3/8)],
                [1.4,2.6],
                [1.1,2.6],
                [1.1 - 0.4*np.sin(np.pi/8),2.6 + 0.4 - 0.4*np.cos(np.pi/8)],
                [1.1 - 0.4*np.sin(np.pi/4),2.6 + 0.4 - 0.4*np.cos(np.pi/4)],
                [1.1 - 0.4*np.sin(np.pi*3/8),2.6 + 0.4 - 0.4*np.cos(np.pi*3/8)],
                [0.7,3.0],
                [0.7 - 0.3 + 0.3*np.cos(np.pi/8),3.0 + 0.3*np.sin(np.pi/8)],
                [0.7 - 0.3 + 0.3*np.cos(np.pi/4),3.0 + 0.3*np.sin(np.pi/4)],
                [0.7 - 0.3 + 0.3*np.cos(np.pi*3/8),3.0 + 0.3*np.sin(np.pi*3/8)],
                [0.4,3.3],
                [0.4 - 0.3*np.sin(np.pi/8),3.3 - 0.3 + 0.3*np.cos(np.pi/8)],
                [0.4 - 0.3*np.sin(np.pi/4),3.3 - 0.3 + 0.3*np.cos(np.pi/4)],
                [0.4 - 0.3*np.sin(np.pi*3/8),3.3 - 0.3 + 0.3*np.cos(np.pi*3/8)],
                [0.1,3.0],
                [0.1 + 0.6*np.sin(np.pi/8),3.0 - 0.6 + 0.6*np.cos(np.pi/8)],
                [0.1 + 0.6*np.sin(np.pi/4),3.0 - 0.6 + 0.6*np.cos(np.pi/4)],
                [0.1 + 0.6*np.sin(np.pi*3/8),3.0 - 0.6 + 0.6*np.cos(np.pi*3/8)],
                [0.7,2.4],
                [0.7,1.9]])

x_goal = y_goal = 0.0
count = 0
torque_M1 = torque_M2 = torque_M3 = 0.0
vel_M1 = vel_M2 = vel_M3 = 0.0

pos_x = pos_y = 0.0
vel_x = vel_y = 0.0
phi = phi_dot = 0.0

alpha = beta = theta = 0.0
alpha_dot = beta_dot = 0.0

prev_angles = None
prev_phi = None

# -------------------------
# Trayectoria
# -------------------------
#N = len(path)
#t = np.linspace(0, 1, N)     # parámetro original
#ts = np.linspace(0, 1, 100)  # parámetro refinado
#N_r = len(ts)
#x_interp = Akima1DInterpolator(t, path[:, 0],method="makima")(ts)
#y_interp = Akima1DInterpolator(t, path[:, 1],method="makima")(ts)

# --------------------------------------------
# Calcular distancias entre puntos consecutivos
# --------------------------------------------
diffs = np.diff(path, axis=0)
segment_lengths = np.linalg.norm(diffs, axis=1)

# --------------------------------------------
# Longitud de arco acumulada
# --------------------------------------------
s = np.concatenate(([0], np.cumsum(segment_lengths)))

# --------------------------------------------
# Parametrizacion
# --------------------------------------------
N = 150
s_uniform = np.linspace(0, s[-1], N)

# --------------------------------------------
# Interpolación makima paramétrica
# --------------------------------------------
x_interp = Akima1DInterpolator(s, path[:, 0], method="makima")(s_uniform)
y_interp = Akima1DInterpolator(s, path[:, 1], method="makima")(s_uniform)

path_uniform = np.column_stack((x_interp, y_interp))

# ==========================
# Funciones auxiliares
# ==========================
def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

# ==========================
# Configuración Serial
# ==========================
ser = serial.Serial(
    port='/dev/ttyAMA0',
    baudrate=115200,
    timeout=1
)

def recibir_datos():
    global pos_x, pos_y, count, N
    if ser.in_waiting > 0:
        try:
            data = ser.readline().decode('utf-8').strip()
            valores = data.split(",")
            pos_x,pos_y,count = map(float, valores)
            #print(f"[DEBUG] Dato crudo: '{data}'")  # Para ver qué llega exactamente
            print(f"[POSICION] X:{pos_x:.2f}, Y:{pos_y:.2f}, {N}, {count}")
            #valores = data.split(",")
            #print(f"[DEBUG] Valores: {valores}")
            
           
        except ValueError as e:
            print(f"[UART] Error en conversión: {e}")
            print(f"[UART] Dato problemático: '{data}'")
        except Exception as e:
            print(f"[UART] Error inesperado: {e}")

        
def enviar_datos():
    global x_interp, y_interp, count, N
    #mensaje = "2.0,0.0\n"
    if count >= N:
        count = N-1

    mensaje = f"{x_interp[int(count)]:.2f},{y_interp[int(count)]:.2f},{N}\n"
    #print(type(mensaje))
    #print(mensaje.encode('utf-8'))
    #mensaje = input(":")
    #print(type(mensaje))
    #mensaje = mensaje + "\n"
    #print(mensaje.encode('utf-8'))
    ser.write(mensaje.encode('utf-8'))
    #print(f"[UART] Enviado: {mensaje.strip()}")

# ==========================
# Configuración IMUs
# ==========================
#i2c = busio.I2C(board.SCL, board.SDA)
#imu1 = adafruit_bno055.BNO055_I2C(i2c, address=0x29)  # Cuaterniones
#imu2 = adafruit_bno055.BNO055_I2C(i2c, address=0x28)  # Euler

time.sleep(1)

# Obtener lecturas iniciales
#quat1_0 = imu1.quaternion
#euler2_0 = imu2.euler

# Esperar lecturas válidas
#while quat1_0 is None or euler2_0 is None:
#    time.sleep(0.1)
#    quat1_0 = imu1.quaternion
#    euler2_0 = imu2.euler

# ---- IMU1: convertir el cuaternión inicial a Euler ----
#q_xyzw_0 = np.array([quat1_0[1], quat1_0[2], quat1_0[3], quat1_0[0]])
#r0 = R.from_quat(q_xyzw_0)
#euler1_0 = r0.as_euler('yzx', degrees=False)

#print("Calibracion inicial completa.")
#print("IMU1 (Euler inicial rad):", np.round(euler1_0, 3))
#print("IMU2 (Euler inicial degree):", euler2_0)

# ==========================
# Lectura de IMUs
# ==========================
#def leer_imus(dt):
#    global alpha, beta, alpha_dot, beta_dot, phi, phi_dot, prev_angles, prev_phi

    # ---- IMU1: usa cuaterniones ----
#    quat1 = imu1.quaternion
#    if quat1 is not None and not np.allclose(quat1, (0.0, 0.0, 0.0, 0.0)):
#        q_xyzw = np.array([quat1[1], quat1[2], quat1[3], quat1[0]])
#        r = R.from_quat(q_xyzw)
#        euler1 = r.as_euler('yzx', degrees=False) # Original planteado: yzx
#        # Restar calibración inicial
#        euler1_rel = euler1 - euler1_0
#        euler1_rel = np.array([wrap_to_pi(a) for a in euler1_rel])
#    else:
#        euler1_rel = np.zeros(3)
#
#    alpha, beta, theta = euler1_rel[0], euler1_rel[1], euler1_rel[2]

#    if prev_angles is not None:
#        alpha_dot = (alpha - prev_angles[0]) / dt
#        beta_dot = (beta - prev_angles[1]) / dt
#    else:
#        alpha_dot = beta_dot = 0.0
#    prev_angles = (alpha, beta)
#
    # ---- IMU2: usa Euler directamente ----
#    euler2 = imu2.euler
#    if euler2 is not None and euler2[0] is not None:
#        euler2_rel = np.radians(np.array(euler2) - np.array(euler2_0))
#        phi = wrap_to_pi(euler2_rel[0])
#    else:
#        phi = 0.0

#    if prev_phi is not None:
#        phi_dot = (phi - prev_phi) / dt
#    else:
#        phi_dot = 0.0
#    prev_phi = phi

    # Debug
#    print(f"[IMU1] alpha={np.degrees(alpha):.2f} beta={np.degrees(beta):.2f} theta = {np.degrees(theta):.2f}| alpha dot={alpha_dot:.3f} beta dot={beta_dot:.3f}")
#    print(f"[IMU2] phi={np.degrees(phi):.2f} | phi dot={phi_dot:.3f}")
#    print(f"[POS] X={pos_x:.2f}, Y={pos_y:.2f}, Vx={vel_x:.2f}, Vy={vel_y:.2f}")
#    print("-" * 70)

# ==========================
# Bucle Principal
# ==========================
dt = 0.02
start = time.time()
while True:
    elapsed = time.time() - start
    #print(elapsed)
    if elapsed >= dt:
        start = time.time()
        
        enviar_datos()
        # 1. UART
        recibir_datos()

        # 2. Cinemática (función externa)
        #vel_x, vel_y, phi_dot = pl.DKinematics(vel_M1, vel_M2, vel_M3, phi)
        #pos_x += dt * vel_x
        #pos_y += dt * vel_y
        #phi   += dt * phi_dot

        # 3. Leer IMUs
        #leer_imus(dt)

        # 4. Enviar comandos
        

