import numpy as np
import time
import serial

# Variables globales para almacenar los datos recibidos
torque_M1 = 0.0 
torque_M2 = 0.0 
torque_M3 = 0.0 
vel_M1 = 0.0 
vel_M2 = 0.0 
vel_M3 = 0.0 

# Configuración del puerto serie
ser = serial.Serial(
    port='/dev/ttyAMA0',  # Para Raspberry Pi 5 (usar '/dev/ttyS0', '/dev/ttyAMA10' o '/dev/serial0')
    baudrate=115200,
    timeout=1)

# Cambiar para len(valores) == 6 al agregar phi y phi_dot
def recibir_datos():
    #global x_var, x_dot, y_var, y_dot, phi, phi_dot
    global torque_M1, torque_M2, torque_M3, vel_M1, vel_M2, vel_M3
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').strip()  # Leer línea completa
        #valores = data.split(",")  # Separar por comas
        print(data)
        #if len(valores) == 6:  ######################## Verificar que sean (cant de variables a leer) valores
        #    try:
                # Convertir y asignar los valores a las variables globales
                #x_var, x_dot, y_var, y_dot = map(float, valores)
                #print(f"Datos recibidos: {x_var}, {x_dot}, {y_var}, {y_dot}, {phi}, {phi_dot}")
        #        torque_M1, torque_M2, torque_M3, vel_M1, vel_M2, vel_M3 = map(float, valores)
                #msg=f"Datos recibidos: {torque_M1}, {torque_M2}, {torque_M3}, {vel_M1}, {vel_M2}, {vel_M3}"
                #ser.write(msg.encode('utf-8'))
        #        print(f"Datos recibidos: {torque_M1}, {torque_M2}, {torque_M3}, {vel_M1}, {vel_M2}, {vel_M3}")

         #   except ValueError:
          #      print("Error en conversión de datos")
        #else:
        #    print("Error: cantidad incorrecta de valores")
        #    print(len(valores))

def enviar_datos(valor1, valor2, valor3):
    mensaje = f"<{valor1},{valor2},{valor3}>\n"  # Crear el mensaje con los valores separados por coma
    ser.write(mensaje.encode('utf-8'))  
    print(f"Enviando: {mensaje}") 

dt = 0.02

while True:
    recibir_datos()
    enviar_datos(0.2, 0.2, 0.2)
    time.sleep(dt)
