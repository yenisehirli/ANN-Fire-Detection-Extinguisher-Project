import cv2
import numpy as np
import socket
from picamera2 import Picamera2
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import pickle
from time import sleep
import math
import os
import RPi.GPIO as GPIO


# GPIO modunu belirle
GPIO.setmode(GPIO.BCM)

greenLed = 14
yellowLed = 15
redLed = 18

# Pin ayarlarını yap
GPIO.setup(greenLed, GPIO.OUT)
GPIO.setup(yellowLed, GPIO.OUT)
GPIO.setup(redLed, GPIO.OUT)

def close_all_leds():
    GPIO.output(greenLed, GPIO.LOW)
    GPIO.output(yellowLed, GPIO.LOW)
    GPIO.output(redLed, GPIO.LOW)

def open_led(pin):
    GPIO.output(pin, GPIO.HIGH)

def close_led(pin):
    GPIO.output(pin, GPIO.LOW)


# Raspberry Pi'nin IP adresi ve kullanacağınız port numarasını buraya yazın
RPI_IP = '0.0.0.0'  # Raspberry Pi'nin IP adresi
PORT = 5001     # Kullanılacak port numarası

MESSAGE_PORT = 12345

# Kamera aç
picam = Picamera2()

config = picam.create_preview_configuration()
picam.configure(config)

factory = PiGPIOFactory()

# servo1 motoru oluştur
servo1 = Servo(13, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory) # Y angle
servo2 = Servo(12, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory) # X angle



# Soket oluştur
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((RPI_IP, PORT))
server_socket.listen(5)

open_led(yellowLed)
print("Bağlantı bekleniyor")

client_socket, address = server_socket.accept()

print("Bağlantı alındı:", address)
print("IP ADRESİ",address[0])

PC_IP = address[0]


pc_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
pc_socket.connect((PC_IP, MESSAGE_PORT))


print("Bilgisayara bağlandı")



open_led(greenLed)

red_flag = 0

picam.start()
try:
    while True:
        try:
        # Kameradan görüntüyü al
            frame = picam.capture_array()
        
        # Görüntüyü baytlara dönüştür
            _, img_encoded = cv2.imencode('.bmp', frame)
            data = np.array(img_encoded)
            string_data = data.tobytes()
       

        # Görüntüyü gönder
            client_socket.sendall((str(len(string_data))).encode().ljust(16) + string_data)
            red_flag +=1
            data = pc_socket.recv(4096)
            if data:
                servo_angles = pickle.loads(data)
                print(servo_angles)
                servo1.value = math.sin(math.radians(servo_angles["Y"]))
                servo2.value = math.sin(math.radians(servo_angles["X"]))

            
            
        except Exception as e:
            print(e)
            # Restart system
            close_all_leds()
         
            break
except KeyboardInterrupt:
    close_all_leds()
    camera.release()
    close_led(greenLed)
    GPIO.cleanup()
    print("\nKullanıcı tarafından işlem iptal edildi.")
    
finally:
    # Kamera ve soketleri kapat
    close_all_leds()
    camera.release()
    close_led(greenLed)
    GPIO.cleanup()

  
  