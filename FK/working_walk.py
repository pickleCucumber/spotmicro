from __future__ import division
import threading
import time
import csv
import math
from adafruit_servokit import ServoKit
from mpu6050 import mpu6050
import numpy as np
from smbus2 import SMBus

#задаем адресс объектов
mpu6050 = mpu6050(address=0x68)
kit0 = ServoKit(address=0x40, channels=16)
kit1 = ServoKit(address=0x41, channels=16)

# объявляем сервомоторы

servo_angles = {
    "FR_clav": 90, "FL_clav": 90, "BL_clav": 90, "BR_clav": 90,
    "FR_hum": 60,  "FL_hum": 60,  "BL_hum": 60,  "BR_hum": 60,
    "FR_rad": 90,  "FL_rad": 90,  "BL_rad": 90,  "BR_rad": 90,
}

OFFSET = {
    "FR_clav": 2,
    "FL_clav": -8,
    "BL_clav": 0,
    "BR_clav": 0,
    "FR_hum": 0,
    "FL_hum": -15,
    "BL_hum": 0,
    "BR_hum": 0,
    "FR_rad": 0,
    "FL_rad": 3,
    "BL_rad": 0,
    "BR_rad": 0,
}


## Правая передняя нога
#ключица
def Front_Right_clauiculum(A):
    kit0.servo[0].angle = 180 - A + OFFSET["FR_clav"]
    servo_angles['FR_clav'] = 180 - A + OFFSET["FR_clav"]
# плечевая
def Front_Right_humerus(A):
    kit0.servo[1].angle = A + OFFSET["FR_hum"]
    servo_angles['FR_hum'] = A + OFFSET["FR_hum"]
# лучевая/локтевая
def Front_Right_radii(A):
    kit0.servo[2].angle = 180 - A + OFFSET["FR_rad"]
    servo_angles['FR_rad'] = 180 - A + OFFSET["FR_rad"]


## Левая передняя нога
#ключица
def Front_Left_clauiculum(A):
    kit0.servo[4].angle = A + OFFSET["FL_clav"]
    servo_angles['FL_clav'] = A + OFFSET["FR_clav"]

#плечевая
def Front_Left_humerus(A):
    kit0.servo[5].angle = 180 - A + OFFSET["FL_hum"]
    servo_angles['FL_hum'] = 180 - A + OFFSET["FL_hum"]
#локтевая
def Front_Left_radii(A):
    kit0.servo[6].angle = A + OFFSET["FL_rad"]
    servo_angles['FL_rad'] =  A + OFFSET["FL_rad"]
    

## Левая задняя нога
#ключица
def Back_Left_clauiculum(A):
    kit1.servo[8].angle = 180 - A + OFFSET["BL_clav"]
    servo_angles['BL_clav'] = 180 - A + OFFSET["BL_clav"]

#плечевая
def Back_Left_humerus(A):
    kit1.servo[9].angle = 180 - A + OFFSET["BL_hum"]
    servo_angles['BL_hum'] =180 - A + OFFSET["BL_hum"]
    
#ключица
def Back_Left_radii(A):
    kit1.servo[10].angle = A + OFFSET["BL_rad"]
    servo_angles['BL_rad'] =  A + OFFSET["BL_rad"]

## Правая задняя нога
#ключица
def Back_Right_clauiculum(A):
    kit1.servo[12].angle = A + OFFSET["BR_clav"]
    servo_angles['BR_clav'] = A + OFFSET["BR_clav"]
    
#плечевая
def Back_Right_humerus(A):
    kit1.servo[13].angle = A + OFFSET["BR_hum"]
    servo_angles['BR_hum'] = A + OFFSET["BR_hum"]

#ключица
def Back_Right_radii(A):
    kit1.servo[14].angle = 180 - A + OFFSET["BR_rad"]
    servo_angles['BR_rad'] = 180 - A + OFFSET["BR_rad"]


#ключицЫ
def collarbone(A):
    Front_Right_clauiculum(A)
    Front_Left_clauiculum(A)
    time.sleep(0.5)
    Back_Left_clauiculum(A)
    Back_Right_clauiculum(A)
    time.sleep(1)

def humerus(A):
    Front_Right_humerus(A)
    Front_Left_humerus(A)
    time.sleep(0.5)
    Back_Left_humerus(A)
    Back_Right_humerus(A)
    time.sleep(1)


def radii(A):
    Front_Right_radii(A)
    Front_Left_radii(A)
    time.sleep(0.5)
    Back_Left_radii(A)
    Back_Right_radii(A)
    time.sleep(1)




#инициализация сервомоторов для калибровки
def Stay():
    print("Инициализация началась")    

    collarbone(90)
    #time.sleep(0.5)

    radii(100)
    #time.sleep(0.5)
    humerus(60)
    time.sleep(1)
    print("Инициализация закончена")
    

def lay():
    radii(0)
    time.sleep(0.5)
    humerus(20)
    time.sleep(0.5)
    collarbone(90)

def sit():
    collarbone(90)
    Front_Right_radii(120)
    Front_Left_radii(120)
    time.sleep(0.5)
    #Back_Left_radii(60)
    Back_Right_radii(60)
    #time.sleep(0.1)
    Back_Left_radii(60)

    time.sleep(0.5)
    Back_Left_humerus(45)
    Back_Right_humerus(45)
    time.sleep(0.5)
    Front_Right_humerus(90)
    Front_Left_humerus(90)

    time.sleep(1)


def Heil():
    #sit()
    
    #Back_Right_radii(60)

    #Front_Left_radii(60)
    Front_Left_clauiculum(130)
    Front_Left_humerus(110)

    Front_Left_radii(160)
    time.sleep(0.2)
    i=0
    while i<7:
        i=i+1
        time.sleep(0.2)
        Front_Right_radii(45)
        time.sleep(0.2)
        Front_Right_radii(90)
    sit()

def ready_for_game():
    collarbone(90)

    Front_Right_humerus(60)
    Front_Left_humerus(60)
    Back_Left_humerus(45)
    Back_Right_humerus(45)
    Front_Right_radii(120)
    Front_Left_radii(120)
    Back_Left_radii(45)
    Back_Right_radii(45)

def log_data():
    accel = mpu6050.get_accel_data()
    gyro = mpu6050.get_gyro_data()
    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')

    row = [
        timestamp,
        accel['x'], accel['y'], accel['z'],
        gyro['x'], gyro['y'], gyro['z'],
        servo_angles['FR_clav'], servo_angles['FL_clav'], servo_angles['BL_clav'], servo_angles['BR_clav'],
        servo_angles['FR_hum'], servo_angles['FL_hum'], servo_angles['BL_hum'], servo_angles['BR_hum'],
        servo_angles['FR_rad'], servo_angles['FL_rad'], servo_angles['BL_rad'], servo_angles['BR_rad']
    ]

    with open('/home/rpi/Desktop/data.csv', mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(row)
    print('записано')


#инициализация файла
def init_csv():
    with open('/home/rpi/Desktop/data.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([
            'timestamp',
            'accel_x', 'accel_y', 'accel_z',
            'gyro_x', 'gyro_y', 'gyro_z',
            'FR_clav', 'FL_clav', 'BL_clav', 'BR_clav',
            'FR_hum', 'FL_hum', 'BL_hum', 'BR_hum',
            'FR_rad', 'FL_rad', 'BL_rad', 'BR_rad'
        ])


def move_forward(step_count=5, step_time=0.5, step_length=15, step_height=30):

    Stay()
    time.sleep(1)
    
    # константы
    neutral_angle = 90
    half_step_time = step_time / 2
    
    for step in range(step_count):
        print(f"Шаг {step + 1}/{step_count}")
        
        # разбиваем шаг на 10 промежутков для плавности
        for i in range(10):
            t = i * 0.1 * step_time
            phase = (t % step_time) < half_step_time
            
            # нормализованное время для фазы
            t_norm = (t % half_step_time) / half_step_time
            
            # вычисляем высоту по параболе (плавный подъем/спуск)
            current_height = 4 * step_height * t_norm * (1 - t_norm)
            
            # фаза 1: FR и BL 
            if phase:
                # передняя правая нога (FR) - смена знака
                clav_angle = neutral_angle - step_length * (2*t_norm - 1)  
                hum_angle = 60 - current_height
                Front_Right_clauiculum(clav_angle)
                Front_Right_humerus(hum_angle)
                
                # Задняя левая нога (BL) - смена знака
                clav_angle = neutral_angle - step_length * (2*t_norm - 1) 
                hum_angle = 60 - current_height
                Back_Left_clauiculum(clav_angle)
                Back_Left_humerus(hum_angle)
                
                # остальные в опорной 
                Front_Left_clauiculum(neutral_angle)
                Front_Left_humerus(60)
                Back_Right_clauiculum(neutral_angle)
                Back_Right_humerus(60)
            
            # фаза 2: FL и BR 
            else:
                # передняя левая нога (FL) -смена знака
                clav_angle = neutral_angle - step_length * (2*t_norm - 1)  
                hum_angle = 60 - current_height
                Front_Left_clauiculum(clav_angle)
                Front_Left_humerus(hum_angle)
                
                # задняя правая нога (BR) - смена знака
                clav_angle = neutral_angle - step_length * (2*t_norm - 1)  
                hum_angle = 60 - current_height
                Back_Right_clauiculum(clav_angle)
                Back_Right_humerus(hum_angle)
                
                # остальные в опорной 
                Front_Right_clauiculum(neutral_angle)
                Front_Right_humerus(60)
                Back_Left_clauiculum(neutral_angle)
                Back_Left_humerus(60)
            
            time.sleep(step_time / 10)
    
    Stay()

#-----------------------------------------------------------------------------------
def smooth_move(servo_func, target_angle, duration):
    """
    Плавное движение сервомотора от текущего угла до целевого.
    :param servo_func: Функция для управления углом сервомотора.
    :param target_angle: Целевой угол для сервомотора.
    :param duration: Время для перехода (в секундах).
    """
    current_angle = servo_angles.get(servo_func.__name__.split('_')[0], 90)  # Начальный угол
    steps = 20  # количество шагов для плавного перехода
    step_time = duration / steps  # время для одного шага
    angle_step = (target_angle - current_angle) / steps  # шаг угла
    
    for i in range(steps):
        new_angle = current_angle + angle_step * (i + 1)
        servo_func(new_angle)
        time.sleep(step_time)
        



def turn_left(step_count=5, step_time=0.5, step_angle=10, step_height=15):
    Stay()
    time.sleep(1)
    
    # константы
    neutral_angle = 90
    half_step_time = step_time / 2
    
    for step in range(step_count):
        print(f"Шаг {step + 1}/{step_count}")
        
        # разбиваем шаг на 10 промежутков для плавности
        for i in range(10):
            t = i * 0.1 * step_time
            phase = (t % step_time) < half_step_time
            
            # нормализованное время для фазы
            t_norm = (t % half_step_time) / half_step_time
            
            # вычисляем высоту по параболе (плавный подъем/спуск)
            current_height = 4 * step_height * t_norm * (1 - t_norm)
            
            # фаза 1: FR и BL (поворот влево)
            if phase:
                # передняя правая нога (FR) - смена знака
                clav_angle = neutral_angle - step_angle * (2 * t_norm - 1)  
                hum_angle = 60 - current_height
                Front_Right_clauiculum(clav_angle)
                Front_Right_humerus(hum_angle)
                
                # Задняя левая нога (BL) - смена знака
                clav_angle = neutral_angle - step_angle * (2 * t_norm - 1) 
                hum_angle = 60 - current_height
                Back_Left_clauiculum(clav_angle)
                Back_Left_humerus(hum_angle)
                
                # остальные в опорной 
                Front_Left_clauiculum(neutral_angle)
                Front_Left_humerus(60)
                Back_Right_clauiculum(neutral_angle)
                Back_Right_humerus(60)
            
            # фаза 2: FL и BR
            else:
                # передняя левая нога (FL) - смена знака
                clav_angle = neutral_angle - step_angle * (2 * t_norm - 1)  
                hum_angle = 60 - current_height
                Front_Left_clauiculum(clav_angle)
                Front_Left_humerus(hum_angle)
                
                # задняя правая нога (BR) - смена знака
                clav_angle = neutral_angle - step_angle * (2 * t_norm - 1)  
                hum_angle = 60 - current_height
                Back_Right_clauiculum(clav_angle)
                Back_Right_humerus(hum_angle)
                
                # остальные в опорной 
                Front_Right_clauiculum(neutral_angle)
                Front_Right_humerus(60)
                Back_Left_clauiculum(neutral_angle)
                Back_Left_humerus(60)
            
            time.sleep(step_time / 10)
    
    Stay()

def turn_right(step_count=5, step_time=0.5, step_angle=15, step_height=15):
    Stay()
    time.sleep(1)
    
    # константы
    neutral_angle = 90
    half_step_time = step_time / 2
    
    for step in range(step_count):
        print(f"Шаг {step + 1}/{step_count}")
        
        # разбиваем шаг на 10 промежутков для плавности
        for i in range(10):
            t = i * 0.1 * step_time
            phase = (t % step_time) < half_step_time
            
            # нормализованное время для фазы
            t_norm = (t % half_step_time) / half_step_time
            
            # вычисляем высоту по параболе (плавный подъем/спуск)
            current_height = 4 * step_height * t_norm * (1 - t_norm)
            
            # фаза 1: FR и BL (поворот вправо)
            if phase:
                # передняя левая нога (FL) - смена знака
                clav_angle = neutral_angle + step_angle * (2 * t_norm - 1)  
                hum_angle = 60 - current_height
                Front_Left_clauiculum(clav_angle)
                Front_Left_humerus(hum_angle)
                
                # Задняя правая нога (BR) - смена знака
                clav_angle = neutral_angle + step_angle * (2 * t_norm - 1) 
                hum_angle = 60 - current_height
                Back_Right_clauiculum(clav_angle)
                Back_Right_humerus(hum_angle)
                
                # остальные в опорной 
                Front_Right_clauiculum(neutral_angle)
                Front_Right_humerus(60)
                Back_Left_clauiculum(neutral_angle)
                Back_Left_humerus(60)
            
            # фаза 2: FL и BR
            else:
                # передняя правая нога (FR) - смена знака
                clav_angle = neutral_angle + step_angle * (2 * t_norm - 1)  
                hum_angle = 60 - current_height
                Front_Right_clauiculum(clav_angle)
                Front_Right_humerus(hum_angle)
                
                # задняя левая нога (BL) - смена знака
                clav_angle = neutral_angle + step_angle * (2 * t_norm - 1)  
                hum_angle = 60 - current_height
                Back_Left_clauiculum(clav_angle)
                Back_Left_humerus(hum_angle)
                
                # остальные в опорной 
                Front_Left_clauiculum(neutral_angle)
                Front_Left_humerus(60)
                Back_Right_clauiculum(neutral_angle)
                Back_Right_humerus(60)
            
            time.sleep(step_time / 10)
    
    Stay()

def Stay():
    print("Инициализация началась")    

    collarbone(90)
    time.sleep(0.5)

    Front_Left_radii(100)
    Back_Right_radii(110)
    time.sleep(0.5)
    Front_Right_radii(100)    #radii(100)
    Back_Left_radii(110)
    time.sleep(0.5)
    Back_Left_humerus(50)
    Front_Right_humerus(60)
    time.sleep(0.5)

    Back_Right_humerus(50)
    Front_Left_humerus(60)

    time.sleep(1)
    print("Инициализация закончена")




def stay_silk2_smooth():
    # ключицы: все одновременно
    threads = [
        threading.Thread(target=lambda: smooth_move(Front_Right_clauiculum, 90, 0.5)),
        threading.Thread(target=lambda: smooth_move(Front_Left_clauiculum, 90, 0.5)),
        threading.Thread(target=lambda: smooth_move(Back_Left_clauiculum, 90, 0.5)),
        threading.Thread(target=lambda: smooth_move(Back_Right_clauiculum, 90, 0.5))
    ]
    for t in threads: t.start()
    for t in threads: t.join()

    # передние локтевые  + задние плечи параллельно
    threads = [
        threading.Thread(target=lambda: smooth_move(Front_Right_radii, 90, 1.0)),
        threading.Thread(target=lambda: smooth_move(Front_Left_radii, 90, 1.0)),
        threading.Thread(target=lambda: smooth_move(Back_Left_humerus, 60, 1.0)),
        threading.Thread(target=lambda: smooth_move(Back_Right_humerus, 60, 1.0))
    ]
    for t in threads: t.start()
    for t in threads: t.join()

    # задние локтевые + передние плечевые параллельно
    threads = [
        threading.Thread(target=lambda: smooth_move(Back_Left_radii, 90, 1.0)),
        threading.Thread(target=lambda: smooth_move(Back_Right_radii, 90, 1.0)),
        threading.Thread(target=lambda: smooth_move(Front_Right_humerus, 60, 1.0)),
        threading.Thread(target=lambda: smooth_move(Front_Left_humerus, 60, 1.0))
    ]
    for t in threads: t.start()
    for t in threads: t.join()

    # фиксируем стойку
    Stay()
#-----------------ТЕСТИЩЕЕЕ----------------------------------------#
def sit_silk2_parallel():

    # ключицы в нейтраль
    smooth_move(Front_Right_clauiculum, 90, 0.5)
    smooth_move(Front_Left_clauiculum, 90, 0.5)
    smooth_move(Back_Left_clauiculum, 90, 0.5)
    smooth_move(Back_Right_clauiculum, 90, 0.5)

    # передние радиусы сгибаем
    t1 = threading.Thread(target=lambda: smooth_move(Front_Right_radii, 120, 1.0))
    t2 = threading.Thread(target=lambda: smooth_move(Front_Left_radii, 120, 1.0))
    for t in [t1, t2]:
        t.start()
    for t in [t1, t2]:
        t.join()
    time.sleep(0.5)

    # задние радиусы разгибаем
    t3 = threading.Thread(target=lambda: smooth_move(Back_Right_radii, 60, 1.0))
    t4 = threading.Thread(target=lambda: smooth_move(Back_Left_radii, 60, 1.0))
    for t in [t3, t4]:
        t.start()
    for t in [t3, t4]:
        t.join()
  



def lay_silk2_parallel():


    # плечи опускаем
    t1 = threading.Thread(target=lambda: smooth_move(Front_Right_humerus, 20, 1.0))
    t2 = threading.Thread(target=lambda: smooth_move(Front_Left_humerus, 20, 1.0))
    t3 = threading.Thread(target=lambda: smooth_move(Back_Left_humerus, 20, 1.0))
    t4 = threading.Thread(target=lambda: smooth_move(Back_Right_humerus, 20, 1.0))

    for t in [t1, t2, t3, t4]:
        t.start()
    for t in [t1, t2, t3, t4]:
        t.join()
    time.sleep(0.5)

    # радиусы вытягиваем
    t5 = threading.Thread(target=lambda: smooth_move(Front_Right_radii, 0, 1.0))
    t6 = threading.Thread(target=lambda: smooth_move(Front_Left_radii, 0, 1.0))
    t7 = threading.Thread(target=lambda: smooth_move(Back_Left_radii, 0, 1.0))
    t8 = threading.Thread(target=lambda: smooth_move(Back_Right_radii, 0, 1.0))

    for t in [t5, t6, t7, t8]:
        t.start()
    for t in [t5, t6, t7, t8]:
        t.join()
    time.sleep(0.5)

    # ключицы в нейтраль
    smooth_move(Front_Right_clauiculum, 90, 0.5)
    smooth_move(Front_Left_clauiculum, 90, 0.5)
    smooth_move(Back_Left_clauiculum, 90, 0.5)
    smooth_move(Back_Right_clauiculum, 90, 0.5)

#------------------------------------------------------------------------#
#плавная ходьба

def move_forward_quadruped_walk(step_count=5, step_time=1.0, step_length=20, step_height=25):
    print(f"Начало походки: {step_count} шагов, {step_time}s на шаг")
    Stay()
    time.sleep(1)

    base_humerus = 60      # базовая стойка
    base_radii = 90        # нейтральная длина лапы
    base_claviculum = 90   # корпус прямо

    # порядок ног и фазовый сдвиг
    legs = [
        ("Front_Right", 0.0),
        ("Front_Left", 0.25),
        ("Back_Left", 0.5),
        ("Back_Right", 0.75)
    ]

    for step in range(step_count):
        step_start = time.time()
        while time.time() - step_start < step_time:
            t = (time.time() - step_start) / step_time

            for leg_name, phase in legs:
                t_leg = (t + phase) % 1.0

                # движение плеча вперёд-назад
                hum_swing = math.sin(2 * math.pi * t_leg) * step_length

                # подъём лапы по параболе
                if t_leg < 0.5:
                    lift_norm = t_leg * 2
                    lift = 4 * step_height * lift_norm * (1 - lift_norm)
                else:
                    lift = 0

                # задние лапы зеркалят передние
                direction = 1 if "Front" in leg_name else -1

                hum_angle = base_humerus + direction * hum_swing
                rad_angle = base_radii - lift * 0.4

                # немного стабилизируем корпус claviculum 
                clav_angle = base_claviculum

                # применяем углы
                if leg_name == "Front_Right":
                    Front_Right_humerus(hum_angle)
                    Front_Right_radii(rad_angle)
                    Front_Right_clauiculum(clav_angle)
                elif leg_name == "Front_Left":
                    Front_Left_humerus(hum_angle)
                    Front_Left_radii(rad_angle)
                    Front_Left_clauiculum(clav_angle)
                elif leg_name == "Back_Left":
                    Back_Left_humerus(hum_angle)
                    Back_Left_radii(rad_angle)
                    Back_Left_clauiculum(clav_angle)
                elif leg_name == "Back_Right":
                    Back_Right_humerus(hum_angle)
                    Back_Right_radii(rad_angle)
                    Back_Right_clauiculum(clav_angle)

            time.sleep(0.02)

    Stay()
    print("Walk done.")
