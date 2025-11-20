#ifndef SIMPLE_PID_H
#define SIMPLE_PID_H

#include <math.h>

// Definición de la estructura para el PID
typedef struct {
    float kp, kd, ki, umax, umin; // Parámetros
    uint32_t control;
    float eprev1, eprev2, sgn; // Variables acumulativas
    volatile float e_int;
    volatile float uk_1, uk_2;
} SimplePID;

// Constructor: Inicializa la estructura con valores por defecto
void SimplePID_Init(SimplePID *pid) {
    pid->kp = 1.0;
    pid->kd = 0.0;
    pid->ki = 0.0;
    pid->umax = 1000.0;
    pid->umin = 10.0;
    pid->eprev1 = 0.0;
    pid->e_int = 0.0;
    pid->sgn = 1.0;
    pid->control = 0.0;
    pid->uk_1 = 0.0;
    pid->uk_2 = 0.0;
    pid->eprev2 = 0.0;
}

// Función para establecer parámetros
void SimplePID_SetParams(SimplePID *pid, float kpIn, float kiIn, float kdIn, float umaxIn, float uminIn) {
    pid->kp = kpIn;
    pid->kd = kdIn;
    pid->ki = kiIn;
    pid->umax = umaxIn;
    pid->umin = uminIn;
}

// Función para calcular la señal de control
void SimpleCPID_Evaluate(SimplePID *pid, volatile float value, float target, float deltaT) {
    // Error
    float e = (target - value);
    
    // Derivada
    float dedt = (e - pid->eprev1) / deltaT ;
    
    // Integral
    float eDt = e*deltaT;

    pid->e_int = pid->e_int + eDt;
    
    // Señal de control
    float u = (pid->kp * e + pid->e_int * pid->ki + pid->kd * dedt)/100*pid->umax;
    pid->sgn = fabs(u)/u;

	// Limitar señal de control
	if (fabs(u) > pid->umax) {
		u = pid->umax;
	}
	else {
		u = u/pid->sgn;
	}
/*
	if (fabs(u)< pid->umin){
		u = pid->umin;
	} else {
		u = u/pid->sgn;
	}
*/
    pid->control = u;

    // Guardar el error anterior
    pid->eprev1 = e;
}

void SimpleDPID_Evaluate(SimplePID *pid, volatile float value, float target, volatile float deltaT, int method) {
    // Error
    float e = (target - value);

    // Constantes
    float beta0  = 0.0;
    float beta1  = 0.0;
    float beta2  = 0.0;
    float alpha1 = 0.0;
    float alpha2 = 0.0;
    if(method == 0.0){ // Discretizacion de  Euler
    	alpha1 = 1.0;
		alpha2 = 0.0;
    	beta0  = pid->kp        + pid->ki*deltaT     + pid->kd/deltaT;
		beta1  = -1.0*pid->kp   + 2.0*pid->kd/deltaT;
		beta2  = pid->kd/deltaT;
    } else{ //Discretizacion Bilineal
    	alpha1 = 0.0;
		alpha2 = 1.0;
    	beta0  = pid->kp        + pid->ki*deltaT/2.0 + 2.0*pid->kd/deltaT;
		beta1  = pid->ki*deltaT - 4.0*pid->kd/deltaT;
		beta2  = pid->kp*(-1.0) + pid->ki*deltaT/2.0 + 2.0*pid->kd/deltaT;
    }

    // Señal de control
    float u = (alpha1*pid->uk_1 + alpha2*pid->uk_2 + beta0*e + beta1*pid->eprev1 + beta2*pid->eprev2)*pid->umax/100.0;
    pid->sgn = fabs(u)/u;

	// Limitar señal de control
	if (fabs(u) > pid->umax) {
		u = pid->umax;
	}
	else {
		u = u/pid->sgn;
	}/*if (fabs(u)< pid->umin){
		u = pid->umin;
	} else {
		u =
	}*/

    pid->control = u;

    // Guardar el varaibles anteriores
    pid->uk_2 = pid-> uk_1;
    pid->uk_1 = u;
    pid->eprev2 = pid->eprev1;
    pid->eprev1 = e;
}

static inline void RungeKutta_4(float *x, float *y, float *phi, float dt, float vx, float vy, float w){

	float k1_x, k2_x, k3_x, k4_x;
	float k1_y, k2_y, k3_y, k4_y;
	float k1_phi, k2_phi, k3_phi, k4_phi;

	k1_x = dt * ( *x + (dt) 		 * (vx));
	k2_x = dt * ( *x + (dt + dt/2.0) * (vx + k1_x/2.0));
	k3_x = dt * ( *x + (dt + dt/2.0) * (vx + k2_x/2.0));
	k4_x = dt * ( *x + (dt + dt) 	 * (vx + k3_x));

	k1_y = dt * ( *y + (dt) 		 * (vy));
	k2_y = dt * ( *y + (dt + dt/2.0) * (vy + k1_y/2.0));
	k3_y = dt * ( *y + (dt + dt/2.0) * (vy + k2_y/2.0));
	k4_y = dt * ( *y + (dt + dt) 	 * (vy + k3_y));

	k1_phi = dt * ( *phi + (dt) 		 * (w));
	k2_phi = dt * ( *phi + (dt + dt/2.0) * (w + k1_phi/2.0));
	k3_phi = dt * ( *phi + (dt + dt/2.0) * (w + k2_phi/2.0));
	k4_phi = dt * ( *phi + (dt + dt) 	 * (w + k3_phi));

	*x = *x + (1.0/6.0) * (k1_x + 2.0*k2_x + 2.0*k3_x + k4_x);
	*y = *y + (1.0/6.0) * (k1_y + 2.0*k2_y + 2.0*k3_y + k4_y);
	*phi = *phi + (1.0/6.0) * (k1_phi + 2.0*k2_phi + 2.0*k3_phi + k4_phi);

}

#endif // SIMPLE_PID_H
