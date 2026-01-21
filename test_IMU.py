import time
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import board
import busio
import adafruit_bno055

# Crear bus I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Crear los dos sensores con direcciones distintas
imu1 = adafruit_bno055.BNO055_I2C(i2c, address=0x28)  # Cuaterniones
imu2 = adafruit_bno055.BNO055_I2C(i2c, address=0x29)  # Euler

time.sleep(1)

# Función para normalizar ángulos a (-pi, pi)
def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

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
euler1_0 = r0.as_euler('yzx', degrees=False)  # orden YZX

print("Calibración inicial completa.")
print("IMU1 (Euler inicial rad):", euler1_0)
print("IMU2 (Euler inicial °):", euler2_0)

# ---- Bucle principal ----
while True:
    quat1 = imu1.quaternion
    euler2 = imu2.euler

    if quat1 is not None and euler2 is not None:
        # --- IMU1: convertir cuaternión a Euler (radianes) ---
        q_xyzw = np.array([quat1[1], quat1[2], quat1[3], quat1[0]])
        r = R.from_quat(q_xyzw)
        euler1 = r.as_euler('yzx', degrees=False)

        # Restar la orientación inicial (para que empiece en 0)
        euler1_rel = euler1 - euler1_0
        euler1_rel = np.array([wrap_to_pi(a) for a in euler1_rel])

        # --- IMU2: convertir Euler de ° a rad y restar offset ---
        euler2_rel = np.radians(np.array(euler2) - np.array(euler2_0))
        euler2_rel = np.array([wrap_to_pi(a) for a in euler2_rel])

        # Mostrar resultados
        print("IMU1 (Euler rad):", np.round(euler1_rel, 3))
        print("IMU2 (Euler rad):", np.round(euler2_rel, 3))
        print("-" * 60)

    time.sleep(0.1)

