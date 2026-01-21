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

# Targets
x_goal = y_goal = 0.0
v_set = [0.0, 0.0, 0.0]
wt = [0.0, 0.0, 0.0]

# 
count = 0
ready_goal = 0
end = 0
flag = 0

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

# -------------------------
# Trayectoria
# -------------------------
# Calculate distance between each consecutive coordinate
# --------------------------------------------
diffs = np.diff(path, axis=0)
segment_lengths = np.linalg.norm(diffs, axis=1)

# --------------------------------------------
# Cumulative arc length
# --------------------------------------------
s = np.concatenate(([0], np.cumsum(segment_lengths)))

# --------------------------------------------
# Parameterization
# --------------------------------------------
N = 50
s_uniform = np.linspace(0, s[-1], N)

# --------------------------------------------
# Parametric makima interpolation
# --------------------------------------------
x_interp = Akima1DInterpolator(s, path[:, 0], method="makima")(s_uniform)
y_interp = Akima1DInterpolator(s, path[:, 1], method="makima")(s_uniform)

path_uniform = np.column_stack((x_interp, y_interp))

# ==========================
# Auxiliar functions
# ==========================
def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

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
            #print(f"[MOTORS] M1:{vel_M1:.2f}|{torque_M1:.2f}, M2:{vel_M2:.2f}|{torque_M2:.2f}, M3:{vel_M3:.2f}|{torque_M3:.2f}")
            
           
        except ValueError as e:
            print(f"[UART] Error en conversión: {e}")
            print(f"[UART] Dato problemático: '{data}'")
        except Exception as e:
            print(f"[UART] Error inesperado: {e}")

        
def enviar_datos():
    global wt, end
    
    mensaje = f"{wt[0]:.2f},{wt[1]:.2f},{wt[2]:.2f},{end}\n"
    ser.write(mensaje.encode('utf-8'))
    #print(f"[UART] Enviado: {mensaje.strip()}")

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
euler1_0 = r0.as_euler('yzx', degrees=False)

#print("Calibracion inicial completa.")
#print("IMU1 (Euler inicial rad):", np.round(euler1_0, 3))
#print("IMU2 (Euler inicial degree):", euler2_0)

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
        euler1 = r.as_euler('yzx', degrees=False) # Original planteado: yzx
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
    #print(f"[IMU1] alpha={np.degrees(alpha):.2f} beta={np.degrees(beta):.2f} theta = {np.degrees(theta):.2f}| alpha dot={alpha_dot:.3f} beta dot={beta_dot:.3f}")
    #print(f"[IMU2] phi={np.degrees(phi):.2f} | phi dot={phi_dot:.3f}")
#    print(f"[POS] X={pos_x:.2f}, Y={pos_y:.2f}, Vx={vel_x:.2f}, Vy={vel_y:.2f}")
    print("-" * 70)

# ==========================
# Bucle Principal
# ==========================
dt = 0.02
goto = pl.GoToGoalController(0.8, 0.0)
stan = pl.StanleyOmni(0.5, 0.05, 0.1) 
pure = pl.PurePursuitOmni3W(0.15, 0.08, 0.4) #distancia, velocidad, k orientacion
controller = pl.StanleyControllerOmnidirectional(
    k_e=1.0,       # Ganancia de error lateral
    k_v=1.0,       # Ganancia de velocidad
    max_steer=0.5  # Límite de velocidad angular
)
pathList = path.tolist()

start = time.time()
while True:
    elapsed = time.time() - start
    #print(elapsed)
    if elapsed >= dt:
        start = time.time()
        
        # 1. UART
        recibir_datos()
        enviar_datos()
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
            if count < N:
                x_goal = x_interp[int(count)]
                y_goal = y_interp[int(count)]
            print(f"estimada: {pos_x, pos_y}")
            print(f"xG:{x_goal:.3f} | yG:{y_goal:.3f} | count: {count}")
            

            #vx_set, vy_set, vw_set = goto.position(pos_x, pos_y, phi, x_goal, y_goal, elapsed) #stanley
            #stanley----
            #x_rel, y_rel = pl.world_to_robot_frame(pos_x, pos_y, phi, x_goal, y_goal)
            #vx_set, vy_set, vw_set = stan.compute(
            #        phi,
            #        (x_goal, y_goal),
            #        (pos_x, pos_y),
            #        (x_rel, y_rel)
            #)

            # Obtener velocidades
            vx_set, vy_set, vw_set = pure.compute(
                robot_pos=(pos_x, pos_y),
                robot_yaw=phi,
                waypoints_list=path     
            )

#-----------------------------------
            wt = pl.IKinematics(vx_set, vy_set, vw_set) #Stanley - pure 
            #print(f"Velocidades deseadas de robot: {vx_set}, {vy_set}, {vw_set}")
            #print(f"Velocidades deseadas a motores: {wt[0]}, {wt[1]}, {wt[2]}")

            if abs(x_goal - pos_x) < 0.1 and abs(y_goal - pos_y) < 0.1:
                 
                if count < N:
                    count = count + 1
                else:
                    wt = [0.0,0.0,0.0]
                    end = 1

            #print(f"Tiempo de procesamiento: {elapsed}")
        

