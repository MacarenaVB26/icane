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


def world_to_robot_frame(robot_x, robot_y, robot_yaw, waypoint_x, waypoint_y):
    dx = waypoint_x - robot_x
    dy = waypoint_y - robot_y
    xc =  dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
    yc = -dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)
    return xc, yc

class StanleyOmni:
    def __init__(self, K_lat=0.4, K_head=0.4, K_w=0.4, V_forward=0.25):
        """
        - K_lat : ganancia sobre error lateral yc
        - K_head: ganancia sobre error de orientación
        - K_w   : ganancia para convertir corrección angular → vw_set
        - V_forward : velocidad hacia adelante del robot
        """
        self.K_lat  = K_lat
        self.K_head = K_head
        self.K_w    = K_w
        self.V_forward = V_forward


    def compute(self, robot_yaw, waypoint_world, robot_world, waypoint_car_frame):
        """
        Calcula vx_set, vy_set, vw_set.
        parámetros:
        robot_yaw          → orientación del robot
        waypoint_world     → (x_goal, y_goal)
        robot_world        → (pos_x, pos_y)
        waypoint_car_frame → (xc, yc) convertido previamente con world_to_robot_frame

        salida:
        vx_set, vy_set, vw_set
        """

        # ----------------------------------------
        # 1) ORIENTACIÓN DEL CAMINO
        # ----------------------------------------
        wx, wy = waypoint_world
        rx, ry = robot_world
        dx = wx - rx
        dy = wy - ry
        path_heading = math.atan2(dy, dx)

        # Normalizador de ángulos
        def norm(a):
            if a > math.pi: a -= 2 * math.pi
            if a < -math.pi: a += 2 * math.pi
            return a

        # ----------------------------------------
        # 2) ERROR DE ORIENTACIÓN
        # ----------------------------------------
        heading_error = norm(path_heading - robot_yaw)
        heading_term = self.K_head * heading_error

        # ----------------------------------------
        # 3) ERROR LATERAL (cross-track)
        # ----------------------------------------
        xc, yc = waypoint_car_frame
        crosstrack_term = math.atan2(self.K_lat * yc, self.V_forward)

        # ----------------------------------------
        # 4) ÁNGULO STANLEY → vw_set
        # ----------------------------------------
        steering_angle = heading_term + crosstrack_term
        vw_set = steering_angle * self.K_w

        # ----------------------------------------
        # 5) VELOCIDADES vx, vy (robot omni)
        # ----------------------------------------
        vx_set = self.V_forward
        vy_set = self.K_lat * yc   # Movimiento lateral para corregir derrape

        return vx_set, vy_set, vw_set