import time
from adafruit_servokit import ServoKit

# Inicializa a placa especificando que ela possui 16 canais [cite: 542, 543]
kit = ServoKit(channels=16) # [cite: 544, 545]

canal0 = 0
canal1 = 1
delay = 2

print(f"--- Servos canal {canal0} e {canal1} ---")

print(f"[Canais {canal0} e {canal1}] Movendo para 0 graus...")
kit.servo[canal0].angle = 0
kit.servo[canal1].angle = 0
time.sleep(delay)

print(f"[Canais {canal0} e {canal1}] Movendo para 90 graus...")
kit.servo[canal0].angle = 90
kit.servo[canal1].angle = 90
time.sleep(delay)

print(f"[Canais {canal0} e {canal1}] Movendo para 180 graus...")
kit.servo[canal0].angle = 180
kit.servo[canal1].angle = 180
time.sleep(delay)

print("Teste finalizado!")