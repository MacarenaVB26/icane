# -*- coding: utf-8 -*-
import time
import math
import serial
import board
import busio
import adafruit_bno055
import numpy as np
from scipy.spatial.transform import Rotation as R
import kinematics_platform as pl
import sys
import csv
from datetime import datetime

# Asegurar codificación UTF-8 para mostrar letras griegas
sys.stdout.reconfigure(encoding='utf-8')

# ==========================
# Variables globales
# ==========================
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
    global torque_M1, torque_M2, torque_M3, vel_M1, vel_M2, vel_M3
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8', errors='replace').strip()
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
    mensaje = f"<{u[0]:.2f},{u[1]:.2f},{u[2]:.2f}>\n"
    ser.write(mensaje.encode('utf-8'))
    #print(f"[UART] Enviado: {mensaje.strip()}")

# ==========================
# Configuración IMUs
# ==========================
i2c = busio.I2C(board.SCL, board.SDA)
imu1 = adafruit_bno055.BNO055_I2C(i2c, address=0x29)  # Cuaterniones
imu2 = adafruit_bno055.BNO055_I2C(i2c, address=0x28)  # Euler

time.sleep(1)

# Obtener lecturas iniciales
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

print("Calibración inicial completa.")
print("IMU1 (Euler inicial rad):", np.round(euler1_0, 3))
print("IMU2 (Euler inicial °):", euler2_0)

# ==========================
# Lectura de IMUs
# ==========================
def leer_imus(dt):
    global alpha, beta, alpha_dot, beta_dot, phi, phi_dot, prev_angles, prev_phi, theta

    # ---- IMU1: usa cuaterniones ----
    quat1 = imu1.quaternion

    if quat1 is not None and not np.allclose(quat1, (0.0, 0.0, 0.0, 0.0)):
        q_xyzw = np.array([-quat1[1], -quat1[2], -quat1[3], -quat1[0]])
        r = R.from_quat(q_xyzw)
        euler1 = r.as_euler('yxz', degrees=False)
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
    print(f"[IMU1] α={np.degrees(alpha):.2f}° β={np.degrees(beta):.2f}° θ={np.degrees(theta):.2f}° | α̇={alpha_dot:.3f} β̇={beta_dot:.3f}")
    print("-" * 70)

# ==========================
# Controlador en espacio de estados
# ==========================
K = np.array([
    [-10, -7.2982,   0.0000, -592.0059, 131.2501, -44.8296, -15.8977, 0.0000, -166.8300, 37.1499],
    [-7, -19.2601,   0.0000, -131.2501, 592.0059, -15.8977, -44.8296, 0.0000,  -37.1499, 166.8300],
    [0.0000, 0.0000, 1.9370, 0.0000, 0.0000, 0.0000, 0.0000, 1.7144, 0.0000, 0.0000]
])

def calculate_control():
    x = np.array([
        pos_x, pos_y, phi, alpha, beta,
        vel_x, vel_y, phi_dot, alpha_dot, beta_dot
    ])
    if K.shape[1] != x.shape[0]:
        print(f"[CONTROL] Error: K tiene {K.shape[1]} columnas y x tiene {x.shape[0]} elementos")
        return np.zeros(3)
    u = -K @ x
    return u

def transform_velocities(u):
    radio = 0.05
    L = 0.15
    phi_c = phi
    eps_O = 30 * np.pi/180
    sigma1 = -np.sin(eps_O + phi_c)
    sigma2 = -np.cos(eps_O + phi_c + np.pi/6)
    sigma3 = np.sin(eps_O + phi_c + np.pi/3)
    sigma4 = np.cos(eps_O + phi_c)
    sigma5 = -np.cos(eps_O + phi_c - np.pi/3)
    sigma6 = -np.cos(eps_O + phi_c + np.pi/3)
    T_f = np.array([[sigma1, sigma2, sigma3],
                    [sigma4, sigma5, sigma6],
                    [L, L, L]])
    f = np.linalg.inv(T_f) @ u
    torq = f * radio
    return torq

# ==========================
# Guardar datos en CSV
# ==========================
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = f"registro_datos_{timestamp}.csv"

with open(filename, mode='w', newline='', encoding='utf-8') as file:
    writer = csv.writer(file)
    writer.writerow([
        "t", 
        "pos_x", "pos_y", "phi", 
        "alpha", "beta", "theta", 
        "vel_x", "vel_y", "phi_dot", "alpha_dot", "beta_dot",
        "u1", "u2", "u3", 
        "torque_M1", "torque_M2", "torque_M3"
    ])

    dt = 0.07
    start = time.time()
    t_global = 0.0

    try:
        while True:
            elapsed = time.time() - start
            if elapsed >= dt:
                start = time.time()
                t_global += dt

                # 1. UART
                recibir_datos()

                # 2. Cinemática
                vel_x, vel_y, phi_dot = pl.DKinematics(vel_M1, vel_M2, vel_M3, phi)
                pos_x += dt * vel_x
                pos_y += dt * vel_y
                phi   += dt * phi_dot

                # 3. Leer IMUs
                leer_imus(dt)

                # 4. Calcular control
                u = calculate_control()
                torque = transform_velocities(u)

                # 5. Enviar comandos
                enviar_datos(torque)

                # 6. Guardar datos
                writer.writerow([
                    round(t_global, 3),
                    pos_x, pos_y, phi,
                    alpha, beta, theta,
                    vel_x, vel_y, phi_dot, alpha_dot, beta_dot,
                    u[0], u[1], u[2],
                    torque_M1, torque_M2, torque_M3
                ])

    except KeyboardInterrupt:
        print("\nFinalizando y guardando datos...")
