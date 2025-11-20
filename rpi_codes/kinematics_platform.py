import numpy as np

# Variables
# Kinematics
EPS_0 = 0
R = 0.05
L = 0.195
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
    ct = np.cos(phi_c)
    st = np.sin(phi_c)
    
    J0 = np.array([[(-2.0/3.0)* (S_eps * ct + st * C_eps), (-2.0/3.0)*(CA_eps_pi6 * ct - st * SA_eps_pi6), (2.0/3.0) *(SA_eps_pi3 * ct + st * CA_eps_pi3)],
                   [(2.0/3.0) * (C_eps * ct - st * S_eps), (-2.0/3.0)*(CM_eps_pi3 * ct - st * SM_eps_pi3), (-2.0/3.0)*(CA_eps_pi3 * ct - st * SA_eps_pi3)]])
    
    vx = R * (vel_M1 * J0[0][0] + vel_M2 * J0[0][1] + vel_M3 * J0[0][2])
    vy = R * (vel_M1 * J0[1][0] + vel_M2 * J0[1][1] + vel_M3 * J0[1][2])
    vw = R * (vel_M1 + vel_M2 + vel_M3) / (3.0 / L)
    
    return vx, vy, vw
