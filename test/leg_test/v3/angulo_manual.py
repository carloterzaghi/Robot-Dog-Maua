import numpy as np
from numpy.linalg import inv, norm
from numpy import asarray, matrix
import math as m
from adafruit_servokit import ServoKit        # type: ignore

# ── Inicialização do controlador de servos (PCA9685, 16 canais, I2C) ──────────
kit = ServoKit(channels=16)
servo_femur = kit.servo[1]   # Theta 2: fêmur — braço 120 mm (Servo Superior do encaixe da perna)
servo_tibia = kit.servo[3]   # Theta 3: tíbia — braço 130 mm (Servo Inferior do encaixe da perna)

def point_to_rad(p1, p2): # converts 2D cartesian points to polar angles in range 0 - 2pi
    theta = m.atan2(p2, p1)
    theta = (theta + 2*m.pi) % (2*m.pi)
    return theta


def angle_corrector(angles=[0,0]):
    # assuming theta_2 = 0 when the leg is pointing down (i.e., 270 degrees offset from the +ve x-axis)
    angles[1] = angles[0] + angles[1] - 5/4*m.pi #theta3 offset
    angles[0] = angles[0] - m.pi/2 #theta2 offset
    
    #Adjusting for angles out of range, and making angles be between -pi,pi
    # for index, theta in enumerate(angles):
    #     if theta > 2*m.pi: angles[index] = np.mod(theta,2*m.pi)
    #     if theta > m.pi: angles[index] = -(2*m.pi -1' theta)
    return angles

def exibe_deg(ang, text, printa):
    if(printa): 
        print(f"\n{text} = {ang} rad -> {ang*180/m.pi} deg")
        return ang*180/m.pi

class Leg_linkage:
    def __init__(self):
        self.upper_leg_length = 120 #mm
        self.lower_leg_length = 130 #mm

        self.a = 35 #mm
        self.b = 32.53 #mm
        self.c = 35 #mm
        self.d = 32.53  #mm

        self.e = 49.5 #mm 11

        self.f = 120 #mm  #new will be 130.0 11
        self.g = 35 #mm 11 
        self.h = 35 #mm 11
        self.i = self.upper_leg_length #mm        

        self.lower_leg_bend_angle = m.radians(0) # degrees found on CAD
        
        self.gamma = m.atan(23/23)
        self.EDC = m.acos((self.c**2+self.h**2-self.e**2)/(2*self.c*self.h))

link = Leg_linkage()

while True:
    print("X:")
    try:
        x = float(input())
        print("Z:")
        z = float(input())

        # x = -15
        # z = -80

        MAX_RADIUS = 220 # mm — raio máximo permitido
        MAX_Z = -80      # mm — profundidade mínima permitida

        if z > MAX_Z:
            z = MAX_Z
            print(f"[AVISO] Z acima do máximo. Ajustado para {MAX_Z} mm")

        len_B = norm([x, 0, z])
        if len_B > MAX_RADIUS:
            scale = MAX_RADIUS / len_B
            x *= scale
            z *= scale
            len_B = MAX_RADIUS
            print(f"[AVISO] Ponto fora do raio máximo. Escalado para ({x:.2f}, {z:.2f})")

        # b_1 : angle between +ve x-axis and len_B (0 <= b_1 < 2pi)
        # b_2 : angle between len_B and link_2
        # b_3 : angle between link_2 and link_3
        b_1 = point_to_rad(x, z)  

        b_2 = m.acos((link.upper_leg_length**2 + len_B**2 -link.lower_leg_length**2) / (2 * link.upper_leg_length * len_B)) 
        b_3 = m.acos((link.upper_leg_length**2 + link.lower_leg_length**2 - len_B**2) / (2 * link.upper_leg_length * link.lower_leg_length))  


        theta_2 = b_1 - b_2
        exibe_deg(theta_2,"theta_2",0)
        theta_3 = m.pi - b_3
        exibe_deg(theta_3,"theta_3",0)


        angulos = [theta_2, theta_3]    

        # modify angles to match robot's configuration (i.e., adding offsets)
        angulos = angle_corrector(angulos)

        servo_femur.angle = exibe_deg(angulos[0],"theta_2 ajustado",1)

        servo_tibia.angle = exibe_deg(angulos[1],"theta_3 ajustado",1) - 8
    except ValueError:
        servo_femur.angle = 90
        servo_tibia.angle = 82
