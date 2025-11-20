import time
import math
import board
import busio
import adafruit_bno055
import numpy as np
from scipy.spatial.transform import Rotation as R

# ==========================
# Función auxiliar
# ==========================
def wrap_to_pi(angle):
    """Envuelve un ángulo al rango [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi

# ==========================
# Configuración IMU (solo IMU1)
# ==========================
i2c = busio.I2C(board.SCL, board.SDA)
imu1 = adafruit_bno055.BNO055_I2C(i2c, address=0x29)

time.sleep(1)

# Esperar lecturas válidas
quat0 = imu1.quaternion
while quat0 is None or all(q == 0 for q in quat0):
    print("Esperando IMU1...")
    time.sleep(0.1)
    quat0 = imu1.quaternion

# Guardar orientación inicial
q_xyzw_0 = np.array([quat0[1], quat0[2], quat0[3], quat0[0]])
r0 = R.from_quat(q_xyzw_0)
euler0 = r0.as_euler('yxz', degrees=False) # Secuencia yxz

print("=== IMU1 listo ===")
print("Orientacion inicial (rad):", np.round(euler0, 3))
print("------------------------------")

# ==========================
# Bucle principal
# ==========================
dt = 0.07
prev_angles = None
alpha = beta = gamma = 0.0
alpha_dot = beta_dot = gamma_dot = 0.0

while True:
    start = time.time()
    euler_default = imu1.euler
    quat = imu1.quaternion
    if quat is not None and not np.allclose(quat, (0.0, 0.0, 0.0, 0.0)):
        # Convertir cuaternión al formato XYZW (Adafruit devuelve WXYZ)
        q_xyzw = np.array([quat[1], quat[2], quat[3], quat[0]])
        r = R.from_quat(q_xyzw)
        euler = r.as_euler('yxz', degrees=False)  # Secuencia yxz

        # Restar orientación inicial (para que empiece en 0)
        euler_rel = euler - euler0
        euler_rel = np.array([wrap_to_pi(a) for a in euler_rel])

        # Asignar ángulos
        alpha, beta, gamma = euler_rel  # α = rotZ, β = rotY, γ = rotX
        alpha = -alpha
        beta = -beta
        # Calcular derivadas (velocidades angulares aproximadas)
        if prev_angles is not None:
            alpha_dot = (alpha - prev_angles[0]) / dt
            beta_dot  = (beta  - prev_angles[1]) / dt
            gamma_dot = (gamma - prev_angles[2]) / dt
        prev_angles = (alpha, beta, gamma)

        # Mostrar en grados (sin símbolos Unicode)
        #print(f"[IMU1] Euler ZYX (deg): alpha={np.degrees(alpha):7.2f}, beta={np.degrees(beta):7.2f}, gamma={np.degrees(gamma):7.2f}")
        print(f"[IMU1] Euler default (deg): alpha={np.degrees(alpha):7.2f}, beta={np.degrees(beta):7.2f}, gamma={np.degrees(gamma):7.2f}")
    else:
        print("[IMU1] Sin lectura válida.")

    # Esperar siguiente ciclo
    elapsed = time.time() - start
    time.sleep(max(0, dt - elapsed))
