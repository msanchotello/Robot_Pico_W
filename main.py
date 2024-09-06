import network
import random
import socket
import time
import machine
import onewire, ds18x20
import ubinascii
import gc
from machine import Pin, PWM, Timer
import utime
import ujson
from time import sleep

ssid = 'Galaxy A53 5G 01C3'
password = 'Miguel123'

autopilot_enabled = False
VELOCIDAD = 500

# Configuración del pin de datos del sensor DS18B20
ds_pin = machine.Pin(15)
ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))
roms = ds_sensor.scan()

# Configuración del servomotor
servo_pin = Pin(16)
servo = PWM(servo_pin)
servo.freq(50)
current_angle = 90  # Ángulo inicial del servomotor

# Función para controlar el servomotor con un ángulo y velocidad lenta
def set_servo_angle_slowly(current_angle, target_angle):
    step = 1 if target_angle > current_angle else -1
    for angle in range(current_angle, target_angle + step, step):
        duty = int((angle / 18.0) + 2.5)
        servo.duty_u16(int(duty * 65536 / 100))
        time.sleep(0.01)  # Retardo para hacer el giro más lento

# Definición de pines para el sensor de distancia
trigger = Pin(4, Pin.OUT)
echo = Pin(5, Pin.IN)

# Definición de pines para motores con PWM
motor1_pwm = PWM(Pin(10))
motor2_pwm = PWM(Pin(11))
motor3_pwm = PWM(Pin(12))
motor4_pwm = PWM(Pin(13))

# Configuración de frecuencia PWM para los motores
motor1_pwm.freq(1000)
motor2_pwm.freq(1000)
motor3_pwm.freq(1000)
motor4_pwm.freq(1000)

# Función para establecer la velocidad de los motores
def set_motor_speed(motor_pwm, speed):
    if speed < 0:
        speed = 0
    elif speed > 1023:
        speed = 1023
    motor_pwm.duty_u16(int(speed * 65536 / 1023))  # Convertimos la velocidad a un valor de 16 bits

# Funciones de control del motor
def forward(speed):
    set_motor_speed(motor1_pwm, 0)
    set_motor_speed(motor2_pwm, speed)
    set_motor_speed(motor3_pwm, speed)
    set_motor_speed(motor4_pwm, 0)

def backward(speed):
    set_motor_speed(motor1_pwm, speed)
    set_motor_speed(motor2_pwm, 0)
    set_motor_speed(motor3_pwm, 0)
    set_motor_speed(motor4_pwm, speed)
    
def left(speed):
    set_motor_speed(motor1_pwm, 0)
    set_motor_speed(motor2_pwm, speed)
    set_motor_speed(motor3_pwm, 0)
    set_motor_speed(motor4_pwm, speed)
    
def right(speed):
    set_motor_speed(motor1_pwm, speed)
    set_motor_speed(motor2_pwm, 0)
    set_motor_speed(motor3_pwm, speed)
    set_motor_speed(motor4_pwm, 0)
  
def stop():
    set_motor_speed(motor1_pwm, 0)
    set_motor_speed(motor2_pwm, 0)
    set_motor_speed(motor3_pwm, 0)
    set_motor_speed(motor4_pwm, 0)

# Conectar a WiFi
def conectar():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)
    while not wlan.isconnected():
        print('Conectando ...')
        time.sleep(1)
    ip = wlan.ifconfig()[0]
    print(f'Conectado con IP: {ip}')
    return ip

# Configurar el socket
def open_socket(ip):
    address = (ip, 80)
    connection = socket.socket()
    connection.bind(address)
    connection.listen(1)
    return connection

# Función para leer la página HTML desde un archivo
def load_html():
    with open('index.html', 'r') as file:
        html = file.read()
    return html

# Función para leer la temperatura
def read_temperature():
    ds_sensor.convert_temp()
    time.sleep_ms(750)
    temp = ds_sensor.read_temp(roms[0])
    return temp

# Función para medir la distancia
def read_distance():
    trigger.low()
    utime.sleep_us(2)
    trigger.high()
    utime.sleep_us(5)
    trigger.low()
    while echo.value() == 0:
        signaloff = utime.ticks_us()
    while echo.value() == 1:
        signalon = utime.ticks_us()
    timepassed = signalon - signaloff
    distance = (timepassed * 0.0343) / 2
    return distance

# Función de control del Autopilot
def autopilot_loop(timer):
    global autopilot_enabled
    if autopilot_enabled:
        dist = read_distance()
        print(dist)

        if dist < 35:
            numero_aleatorio = random.uniform(0, 2)
            print("Girar:", numero_aleatorio, "segundos")
            numero = random.randint(1, 2)
            if numero == 1:
                right(VELOCIDAD)
                sleep(numero_aleatorio)
                stop()
                sleep(0.1)
            else:
                left(VELOCIDAD)
                sleep(numero_aleatorio)
                stop()
                sleep(0.1)
        else:
            forward(VELOCIDAD)
            sleep(0.1)
    else:
        stop()

# Configurar un temporizador para ejecutar el autopilot en intervalos de tiempo
autopilot_timer = Timer(-1)

# Servidor
def serve(connection):
    global current_angle, autopilot_enabled
    speed = 600  # Velocidad inicial
    while True:
        cliente = connection.accept()[0]
        peticion = cliente.recv(1024)
        peticion = str(peticion)
        print('Petición:', peticion)  # Añadir impresión para depuración

        try:
            peticion = peticion.split()[1]
        except IndexError:
            continue
        
        print('Procesando:', peticion)  # Añadir impresión para depuración
        
        if '/autopilot' in peticion:
            if 'state=on' in peticion:
                autopilot_enabled = True
                autopilot_timer.init(period=500, mode=Timer.PERIODIC, callback=autopilot_loop)
                print('Autopilot activado')  # Activa el Autopilot
            elif 'state=off' in peticion:
                autopilot_enabled = False
                autopilot_timer.deinit()  # Detener el temporizador
                stop()  # Detenemos el vehículo si se desactiva el Autopilot
                print('Autopilot desactivado')
            
            # Responder al cliente que la acción se ha realizado
            cliente.send('HTTP/1.1 200 OK\n')
            cliente.send('Content-Type: text/plain\n')
            cliente.send('Connection: close\n\n')
            cliente.sendall('Autopilot activado' if autopilot_enabled else 'Autopilot desactivado')
            cliente.close()
            continue
        
        if '/sensor_data' in peticion:
            temperature = read_temperature()
            distance = read_distance()
            
            # Crear una respuesta JSON con los datos de los sensores
            sensor_data = {
                'temperature': temperature,
                'distance': distance
            }
            
            # Enviar la respuesta JSON
            cliente.send('HTTP/1.1 200 OK\n')
            cliente.send('Content-Type: application/json\n')
            cliente.send('Connection: close\n\n')
            cliente.sendall(ujson.dumps(sensor_data))
            cliente.close()
            continue  # Salir de este ciclo y esperar la próxima solicitud               
            
                            
        if '/cam_left' in peticion:
            current_angle = max(current_angle - 10, 0)  # Disminuye el ángulo para girar a la izquierda
            set_servo_angle_slowly(current_angle + 10, current_angle)
        elif '/cam_right' in peticion:
            current_angle = min(current_angle + 10, 180)  # Aumenta el ángulo para girar a la derecha
            set_servo_angle_slowly(current_angle - 10, current_angle)
        elif '/cam_center' in peticion:
            servo.duty_u16(0)

        if '/motor_forward' in peticion:
            forward(speed)
        elif '/motor_backward' in peticion:
            backward(speed)
        elif '/motor_left' in peticion:
            left(speed)
        elif '/motor_right' in peticion:
            right(speed)
        elif '/motor_stop' in peticion:
            stop()

        if '/speed' in peticion:
            try:
                new_speed = int(peticion.split('=')[-1])
                print('Nueva velocidad:', new_speed)  # Añadir impresión para depuración
                speed = new_speed
            except ValueError:
                pass

        temperature = read_temperature()
        distance = read_distance()
        
        response = load_html()
        response = response.replace('TemperatureLoading', f'{temperature:.2f}')
        response = response.replace('DistanceLoading', f'{distance:.2f} cm')
        
        cliente.send('HTTP/1.1 200 OK\n')
        cliente.send('Content-Type: text/html\n')
        cliente.send('Connection: close\n\n')
        cliente.sendall(response)
        cliente.close()

# Ejecución
try:
    ip = conectar()
    connection = open_socket(ip)
    serve(connection)
except KeyboardInterrupt:
    machine.reset()

