from machine import Pin, PWM
import time

# Definición de estados
#WAIT_ACCESS = En este estado se espera que un auto sea detectado por el sensor ultrasonico para entrar
#CAR_ARRIVAL = En este estado se espera que el auto este dentro del estacionamiento una vez que la barra se haya levantado 
#RAISE_ARM = Este estado levanta el brazo del acceso del estacionamiento
#LOWER_ARM = Este estado baja el brazo del acceso del estacionamiento
#WAIT_EXIT = En este estado se espera que un auto sea detectado por el sensor ultrasonico para salir
#CAR_EXIT = En este estado se espera que el auto este fuera del estacionamiento una vez que la barra se haya levantado 
class State:
    WAIT_ACCESS = 0
    CAR_ARRIVAL = 1
    RAISE_ARM = 2
    LOWER_ARM = 3
    WAIT_EXIT = 4
    CAR_EXIT = 5

# Configuración de los pines ----------------
#Boton
button1 = Pin(18, Pin.IN)

#Pines del sensor ultrasonico
trig = Pin(1, Pin.OUT)
echo = Pin(2, Pin.IN, Pin.PULL_DOWN)

#Pines de los servomotores
servo_pin = PWM(Pin(3))
servo2_pin = PWM(Pin(4))

#Pines de los leds
red_led = Pin(16, Pin.OUT)
green_led = Pin(15, Pin.OUT)

#Pines del display 7 segmentos
e_disp = Pin(11, Pin.OUT)
d_disp = Pin(12, Pin.OUT)
c_disp = Pin(13, Pin.OUT)
g_disp = Pin(22, Pin.OUT)
f_disp = Pin(21, Pin.OUT)
a_disp = Pin(20, Pin.OUT)
b_disp = Pin(19, Pin.OUT)

# Configuración del PWM para el servo
servo_pin.freq(50)
servo2_pin.freq(50)

servo_up = 0 #Left = 1, Right = 0

# Estado inicial
current_state = State.WAIT_ACCESS

# Posiciones de los servos en grados
left_position = 0
right_position = 180

#Maxima capacidad del estacionamiento
max_limit = 5

#Numero de coches en el estacionamiento
num_cars = 0

#Distancia (cm) del auto detectado por el sensor ultrasonico
distance = 0

#Distancia (cm) maxima para detectar los coches
distance_detect = 8

#Variable que gestiona el modo de acceso (0.- Entrada, 1.- Salida)
access_mode = 0

#Valor de los leds
#Led verde = led_value
#Led rojo = not led_value
led_value = 1


# Esta funcion setea el angulo del servomotor
def set_servo_angle(position, servoPin):
    min_duty = 1000  # Ciclo de trabajo mínimo (correspondiente a 0 grados)
    max_duty = 9000  # Ciclo de trabajo máximo (correspondiente a 180 grados)
    
    duty = int(min_duty + (max_duty - min_duty) * (position / 180))
    servoPin.duty_u16(duty)
    #print(f"Moved servo to {position} degrees (duty: {duty})")


#Esta funcion mueve el servomotor
def move_servo(up , servoPin):
    inc = 1 if up else -1
    mini = 0 if up else 90
    maxi = 90 if up else -1
    
    for angle in range(mini, maxi, inc):
        set_servo_angle(angle, servoPin)
        time.sleep(0.01)


#Esta funcion obtiene informacion del sensor ultrasonico
def sensor_ultrasonico():
    trig.value(0)
    time.sleep(0.1)
    trig.value(1)
    time.sleep_us(2)
    trig.value(0)
    while echo.value()==0:
         pulse_start = time.ticks_us()
    while echo.value()==1:
         pulse_end = time.ticks_us()
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17165 / 1000000
    distance = round(distance, 0)
    return distance

#---- Las siguientes funciones son las funciones de los estados -----
def wait_access_func() -> State:
    global access_mode
    if not button1.value():
        access_mode = 1
        move_servo(False, servo2_pin)
        return State.WAIT_EXIT
    distance = sensor_ultrasonico()
    if distance < distance_detect and num_cars < max_limit:
        move_servo(False, servo2_pin)
        return State.RAISE_ARM
    return State.WAIT_ACCESS
    
def car_arrival_func() -> State:
    global num_cars
    distance = sensor_ultrasonico()
    if distance < distance_detect:
        num_cars += 1
        move_servo(True, servo2_pin)
        print(f"Carro estacionado, numero de carros: {num_cars}")
        return State.LOWER_ARM
    return State.CAR_ARRIVAL

def raise_arm_func() -> State:
    move_servo(False, servo_pin)
    return State.CAR_EXIT if access_mode else State.CAR_ARRIVAL

def lower_arm_func() -> State:
    move_servo(True, servo_pin)
    return State.WAIT_EXIT if access_mode else State.WAIT_ACCESS

def wait_exit_func() -> State:
    global access_mode
    if not button1.value():
        access_mode = 0
        move_servo(True, servo2_pin)
        return State.WAIT_ACCESS
    distance = sensor_ultrasonico()
    if distance < distance_detect and num_cars > 0:
        move_servo(True, servo2_pin)
        return State.RAISE_ARM
    return State.WAIT_EXIT

def car_exit_func() -> State:
    global num_cars
    distance = sensor_ultrasonico()
    if distance < distance_detect:
        num_cars -= 1
        move_servo(False, servo2_pin)
        print(f"Carro salio del estacionamiento, numero de carros: {num_cars}")
        return State.LOWER_ARM
    return State.CAR_EXIT
    
#-----------------------------------------------------------------

#--- Funciones para mostrar numeros en el display ----------------
def display_0():
    a_disp.value(1)
    b_disp.value(1)
    c_disp.value(1)
    d_disp.value(1)
    e_disp.value(1)
    f_disp.value(1)
    g_disp.value(0)

def display_1():
    b_disp.value(1)
    c_disp.value(1)
    a_disp.value(0)
    f_disp.value(0)
    g_disp.value(0)
    e_disp.value(0)
    d_disp.value(0)
    
def display_2():
    a_disp.value(1)
    b_disp.value(1)
    g_disp.value(1)
    e_disp.value(1)
    d_disp.value(1)
    f_disp.value(0)
    c_disp.value(0)
    
def display_3():
    a_disp.value(1)
    b_disp.value(1)
    g_disp.value(1)
    c_disp.value(1)
    d_disp.value(1)
    f_disp.value(0)
    e_disp.value(0)
    
def display_4():
    f_disp.value(1)
    b_disp.value(1)
    g_disp.value(1)
    c_disp.value(1)
    a_disp.value(0)
    e_disp.value(0)
    d_disp.value(0)
    
def display_5():
    a_disp.value(1)
    f_disp.value(1)
    g_disp.value(1)
    c_disp.value(1)
    d_disp.value(1)
    b_disp.value(0)
    e_disp.value(0)
#-----------------------------------------------------------

#--- Diccionarios de funciones -----------------------------
states_funcs = {
    State.WAIT_ACCESS: wait_access_func,
    State.CAR_ARRIVAL: car_arrival_func,
    State.RAISE_ARM: raise_arm_func,
    State.LOWER_ARM: lower_arm_func,
    State.WAIT_EXIT: wait_exit_func,
    State.CAR_EXIT: car_exit_func,
}

display_nums = {
    0: display_0,
    1: display_1,
    2: display_2,
    3: display_3,
    4: display_4,
    5: display_5
}
#------------------------------------------------------------

while True:
    current_state = states_funcs[current_state]()
    display_nums[num_cars]()
    led_value = 1 if num_cars < max_limit else 0
    green_led.value(led_value)
    red_led.value(not led_value)
    print(current_state)
