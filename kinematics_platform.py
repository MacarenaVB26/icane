import numpy as np
import math

# Variables
# Kinematics
EPS_0 = 0
R = 0.05
L = 0.195
J0 = np.zeros((2, 3))
C_eps = np.cos(EPS_0)
S_eps = np.sin(EPS_0)
CA_eps_pi6 = np.cos(EPS_0 + np.pi/6)
CM_eps_pi3 = np.cos(EPS_0 - np.pi/3)
CA_eps_pi3 = np.cos(EPS_0 + np.pi/3)
SA_eps_pi6 = np.sin(EPS_0 + np.pi/6)
SM_eps_pi3 = np.sin(EPS_0 - np.pi/3)
SA_eps_pi3 = np.sin(EPS_0 + np.pi/3)

#Functions

def DKinematics(vel_M1, vel_M2, vel_M3, phi_c):

    global J0
    ct = np.cos(phi_c)
    st = np.sin(phi_c)
    
    J0 = np.array([[(-2.0/3.0)* (S_eps * ct + st * C_eps), (-2.0/3.0)*(CA_eps_pi6 * ct - st * SA_eps_pi6), (2.0/3.0) *(SA_eps_pi3 * ct + st * CA_eps_pi3)],
                   [(2.0/3.0) * (C_eps * ct - st * S_eps), (-2.0/3.0)*(CM_eps_pi3 * ct - st * SM_eps_pi3), (-2.0/3.0)*(CA_eps_pi3 * ct - st * SA_eps_pi3)]])
    
    vx = R * (vel_M1 * J0[0][0] + vel_M2 * J0[0][1] + vel_M3 * J0[0][2])
    vy = R * (vel_M1 * J0[1][0] + vel_M2 * J0[1][1] + vel_M3 * J0[1][2])
    vw = R * (vel_M1 + vel_M2 + vel_M3) / (3.0 / L)
    
    return vx, vy, vw
def transform_velocities(u, phi):
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

def IKinematics(vx_set, vy_set, vw_set):
    """
    Function that computes the velocity in rpm and the direction
    of each wheel from the absolute velocity using NumPy.

    Inputs:
        - vx_set: Linear velocity in X axis, in m/s.
        - vy_set: Linear velocity in Y axis, in m/s.
        - vw_set: Angular velocity in Z axis, in rad/s.
    
    Returns:
        - vt: Array of angular velocities for each wheel in rad/s
    """
    global J0

    JI = (3.0 / 2.0) * J0.T  
    
    # Velocity vector
    v = np.array([vx_set, vy_set])
    
    # Calculate angular velocity of each motor in rad/s
    w = (JI @ v + L * vw_set) / R
    
    return w

class GoToGoalController:
    def __init__(self, kp=0.5, ki=0.0):
        """
        Controlador Go to Goal con control PI
        
        Args:
            kp: Ganancia proporcional
            ki: Ganancia integral
        """
        self.kp = kp
        self.ki = ki
        
        # Integrales acumuladas
        self.eint_GGX = 0.0
        self.eint_GGY = 0.0
        
        # Bandera de listo
        self.ready = 0
    
    def position(self, x, y, theta, xGoal_l, yGoal_l, dt):
        """
        Calcula las velocidades vx, vy, vw para alcanzar el objetivo
        
        Args:
            x: Posición actual en X (m)
            y: Posición actual en Y (m)
            theta: Orientación actual (rad)
            xGoal_l: Objetivo en X (m)
            yGoal_l: Objetivo en Y (m)
            dt: Tiempo transcurrido (s)
        
        Returns:
            vx_set: Velocidad en X del robot (m/s)
            vy_set: Velocidad en Y del robot (m/s)
            vw_set: Velocidad angular (rad/s)
        """
        # Calcular errores
        if self.ready == 1:
            errorX = 0.0
            errorY = 0.0
            self.ready = 0
        else:
            errorX = xGoal_l - x
            errorY = yGoal_l - y
        
        # Control Proporcional
        propX = errorX * self.kp
        propY = errorY * self.kp
        
        # Control Integral
        self.eint_GGX += errorX * dt
        integralX = self.eint_GGX * self.ki
        
        self.eint_GGY += errorY * dt
        integralY = self.eint_GGY * self.ki
        
        # Señal de control en marco global
        vx_global = propX + integralX
        vy_global = propY + integralY
        
        # Sin velocidad angular
        vw_set = 0.0
        
        # Calcular velocidad relativa para el robot (transformación de marco)
        ct = np.cos(theta)
        st = np.sin(theta)
        
        vx_set = ct * vx_global + st * vy_global
        vy_set = -st * vx_global + ct * vy_global
        
        return vx_set, vy_set, vw_set
    
    def reset_integrals(self):
        """Reinicia los acumuladores integrales"""
        self.eint_GGX = 0.0
        self.eint_GGY = 0.0


import math

def world_to_robot_frame(robot_x, robot_y, robot_yaw, waypoint_x, waypoint_y):
    """
    Convierte un punto del mundo al frame del robot.
    Devuelve:
    xc → distancia hacia adelante
    yc → error lateral (cross-track)
    """
    dx = waypoint_x - robot_x
    dy = waypoint_y - robot_y

    xc = dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
    yc = -dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)

    return xc, yc

class StanleyControllerOmnidirectional:
    def __init__(self, k_e=0.5, k_v=1.0, k_soft=0.1, max_steer=1.0):
        """
        Stanley Controller para robot omnidireccional de 3 ruedas
        
        Parámetros:
        - k_e: ganancia del error de distancia lateral (crosstrack error)
        - k_v: ganancia de velocidad lineal
        - k_soft: constante de suavizado para evitar división por cero
        - max_steer: límite de steering angular máximo (rad/s)
        """
        self.k_e = k_e
        self.k_v = k_v
        self.k_soft = k_soft
        self.max_steer = max_steer
    
    def world_to_robot_frame(self, robot_x, robot_y, robot_yaw, point_x, point_y):
        """Transforma punto del mundo al frame del robot"""
        dx = point_x - robot_x
        dy = point_y - robot_y
        xc = dx * math.cos(robot_yaw) - dy * math.sin(robot_yaw)
        yc = dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)
        return xc, yc
    
    def find_nearest_point(self, robot_x, robot_y, path):
        """
        Encuentra el punto más cercano en el path y su índice
        
        Returns:
            nearest_point: (x, y, idx, heading)
        """
        min_dist = float('inf')
        nearest_idx = 0
        
        for i, (px, py) in enumerate(path):
            dist = math.hypot(px - robot_x, py - robot_y)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        
        # Calcular el heading del path en ese punto
        if nearest_idx < len(path) - 1:
            # Usar el siguiente punto para calcular dirección
            dx = path[nearest_idx + 1][0] - path[nearest_idx][0]
            dy = path[nearest_idx + 1][1] - path[nearest_idx][1]
            path_heading = math.atan2(dy, dx)
        else:
            # Último punto: usar punto anterior
            if nearest_idx > 0:
                dx = path[nearest_idx][0] - path[nearest_idx - 1][0]
                dy = path[nearest_idx][1] - path[nearest_idx - 1][1]
                path_heading = math.atan2(dy, dx)
            else:
                path_heading = 0.0
        
        nearest_x = path[nearest_idx][0]
        nearest_y = path[nearest_idx][1]
        
        return nearest_x, nearest_y, nearest_idx, path_heading
    
    def normalize_angle(self, angle):
        """Normaliza ángulo entre -pi y pi"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def compute_crosstrack_error(self, robot_x, robot_y, robot_yaw, 
                                 nearest_x, nearest_y, path_heading):
        """
        Calcula el error de crosstrack (distancia lateral al path)
        
        Returns:
            crosstrack_error: error lateral con signo
        """
        # Vector del punto más cercano al robot
        dx = robot_x - nearest_x
        dy = robot_y - nearest_y
        
        # Calcular error perpendicular al path
        # Positivo si el robot está a la izquierda del path
        crosstrack_error = -dx * math.sin(path_heading) + dy * math.cos(path_heading)
        
        return crosstrack_error
    
    def compute_control(self, robot_x, robot_y, robot_yaw, path, target_velocity=0.5):
        """
        Calcula velocidades de control usando Stanley Controller
        
        Parámetros:
        - robot_x, robot_y: posición del robot en mundo
        - robot_yaw: orientación del robot (radianes)
        - path: lista de puntos [[x1,y1], [x2,y2], ...]
        - target_velocity: velocidad lineal deseada (m/s)
        
        Returns:
            vx_robot: velocidad frontal del robot (m/s)
            vy_robot: velocidad lateral del robot (m/s)
            omega: velocidad angular (rad/s)
        """
        if len(path) < 2:
            return 0.0, 0.0, 0.0
        
        # 1. Encontrar punto más cercano en el path
        nearest_x, nearest_y, idx, path_heading = self.find_nearest_point(
            robot_x, robot_y, path
        )
        
        # Verificar si llegamos al final del path
        distance_to_end = math.hypot(
            path[-1][0] - robot_x, 
            path[-1][1] - robot_y
        )
        if distance_to_end < 0.1:  # Umbral de llegada
            return 0.0, 0.0, 0.0
        
        # 2. Calcular crosstrack error (error lateral)
        crosstrack_error = self.compute_crosstrack_error(
            robot_x, robot_y, robot_yaw,
            nearest_x, nearest_y, path_heading
        )
        
        # 3. Calcular heading error (error angular)
        heading_error = self.normalize_angle(path_heading - robot_yaw)
        
        # 4. Stanley Control Law
        # omega = heading_error + atan(k * crosstrack_error / v)
        
        # Velocidad efectiva (evitar división por cero)
        v_eff = max(abs(target_velocity), self.k_soft)
        
        # Término de corrección por crosstrack error
        crosstrack_term = math.atan2(self.k_e * crosstrack_error, v_eff)
        
        # Velocidad angular total
        omega = heading_error + crosstrack_term
        
        # Limitar steering
        omega = max(-self.max_steer, min(self.max_steer, omega))
        
        # 5. Velocidad lineal
        # Reducir velocidad en curvas cerradas
        speed_factor = 1.0 / (1.0 + abs(omega) * 0.5)
        vx_robot = self.k_v * target_velocity * speed_factor
        
        # 6. Velocidad lateral (opciones)
        # Opción A: Sin movimiento lateral (comportamiento tipo Ackermann)
        vy_robot = 0.0
        
        # Opción B: Corrección lateral para aprovechar omnidireccionalidad
        # vy_robot = -0.3 * crosstrack_error  # Factor pequeño para suavizar
        
        return vx_robot, vy_robot, omega




class StanleyOmni:
    def __init__(self, K_e, K_head, V_nominal):
        """
        Controlador Stanley adaptado a robot omnidireccional.

        K_e  → ganancia del error lateral
        K_head    → ganancia del error de orientación (vw)
        V_nominal → velocidad nominal usada en atan2 (NO velocidad real)
        """
        self.K_e = K_e
        self.K_head = K_head
        self.V_nominal = V_nominal

    def compute(self, robot_yaw, waypoint_world, robot_world, waypoint_car_frame):
        """
        Calcula velocidades de referencia:
        vx_set, vy_set, vw_set
        """

        wx, wy = waypoint_world
        rx, ry = robot_world

        # ----------------------------------------
        # 1) Heading del camino (path heading)
        # ----------------------------------------
        path_heading = math.atan2(wy - ry, wx - rx)

        # Normalización de ángulos a [-pi, pi]
        def normalize(angle):
            if angle > math.pi:
                angle -= 2 * math.pi
            elif angle < -math.pi:
                angle += 2 * math.pi
            return angle

        # ----------------------------------------
        # 2) Error de orientación
        # ----------------------------------------
        heading_error = normalize(path_heading - robot_yaw)

        # ----------------------------------------
        # 3) Error lateral (cross-track)
        # ----------------------------------------
        _, yc = waypoint_car_frame

        # ----------------------------------------
        # 4) Ley de control Stanley
        # θ = φ + atan(Ke * e / v)
        # ----------------------------------------
        stanley_term =  (self.K_head * heading_error + math.atan2(self.K_e * yc, self.V_nominal))
        
        # ----------------------------------------
        # 5) Velocidades lineales (omni)
        # ----------------------------------------
        vx_set = self.V_nominal          # avance base
        vy_set = 0.0       #stanley no aplica correccion en velocidad lateral
        vw_set = stanley_term

        return vx_set, vy_set, vw_set

class PurePursuitOmni3W:
    def __init__(self, lookahead_distance, V_nominal, K_heading):
        """
        - lookahead_distance: distancia mínima al waypoint para seleccionarlo como objetivo
        - V_nominal: velocidad lineal base del robot
        - K_heading: ganancia para corregir la orientación hacia el lookahead point
        """
        self.L_d = lookahead_distance
        self.V_nominal = V_nominal
        self.K_heading = K_heading
        self.last_idx = 0  

    def compute(self, robot_pos, robot_yaw, waypoints_list):
       
        rx, ry = robot_pos

        # -----------------------------
        # 1) Seleccionar el lookahead point
        # -----------------------------
        lookahead_point = waypoints_list[-1]  # por defecto último
        for i in range(self.last_idx, len(waypoints_list)):
            p = waypoints_list[i]
            dist = math.hypot(p[0]-rx, p[1]-ry)
            if dist >= self.L_d:
                lookahead_point = p
                self.last_idx = i  # actualizar índice
                break
            
        lx, ly = lookahead_point

        # -----------------------------
        # 2) Transformar el waypoint al frame del robot
        # -----------------------------
        xc, yc = world_to_robot_frame(rx, ry, robot_yaw, lx, ly)

        # -----------------------------
        # 3) Velocidades en el frame local
        # vx_set: velocidad hacia adelante
        # vy_set: velocidad lateral (para corregir el cross-track)
        # -----------------------------
        distance = math.hypot(xc, yc)
        print(f"Lookahead point: ({lx:.3f}, {ly:.3f})")
        print(f"Error en frame robot: xc={xc:.3f}, yc={yc:.3f}, distancia={distance:.3f}")

        if distance == 0:
            distance = 1e-6  # evitar división por cero

        # Movimiento proporcional hacia el lookahead point
        vx_set = self.V_nominal * xc / distance
        vy_set = self.V_nominal * yc / distance

        # -----------------------------
        # 4) Corrección de orientación (vw_set)
        # -----------------------------
        path_heading = math.atan2(yc, xc)
        vw_set = self.K_heading * path_heading
        print(f"path heading: {path_heading}")

        return vx_set, vy_set, vw_set
