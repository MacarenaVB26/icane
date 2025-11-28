import numpy as np

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
