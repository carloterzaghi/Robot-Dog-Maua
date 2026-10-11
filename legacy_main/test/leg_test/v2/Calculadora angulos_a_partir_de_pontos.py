import numpy as np
from numpy.linalg import inv, norm
from numpy import asarray, matrix
import math as m


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
    #     if theta > m.pi: angles[index] = -(2*m.pi - theta)
    return angles

def exibe_deg(ang, text, printa):
    if(printa): print(f"\n{text} = {ang} rad -> {ang*180/m.pi} deg")

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


print("X:")
x = float(input())
print("Z:")
z = float(input())

# x = -15
# z = -80

len_B = norm([x, 0, z])

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

exibe_deg(angulos[0],"theta_2 ajustado",1)

exibe_deg(angulos[1],"theta_3 ajustado",1)







# def lower_leg_angle_to_servo_angle(link, THETA2, THETA3):
#     ''' Converts the direct angles of the upper and lower leg joint from the inverse kinematics to the angle
#     at the servo that drives the lower leg via a linkage. 
#     Parameters
#         ----------
#     THETA2 : float
#         angle of upper leg from the IK 
#     THETA3 : float
#         angle of lower leg from the IK 
#     link: Leg_linage
#         A linkage class with all link lengths and relevant angles stored. Link values are based off
#         the physcial design of the link

#     Returns
#     -------
#     THETA0: float
#         The angle of the servo that drives the outside of the linkage
#     '''

#     # First 4 bar linkages
#     GDE,DEF,EFG = calculate_4_bar(THETA3 + link.lower_leg_bend_angle,link.i,link.h,link.f,link.g) #+ link.lower_leg_bend_angle

#     # Triangle section
#     CDH = 3/2*m.pi - THETA2 - GDE - link.EDC
#     CDA = CDH +link.gamma #input angle
    
#     # Second 4 bar linkage
#     DAB,ABC,BCD = calculate_4_bar(CDA ,link.d,link.a,link.b,link.c)

#     #Calculating Theta
#     THETA0 = DAB + link.gamma

#     return THETA0


# def calculate_4_bar(th2 ,a,b,c,d):
#     """Using 'Freudensteins method', it finds all the angles within a 4 bar linkage with vertices ABCD and known link lengths a,b,c,d
#     defined clockwise from point A, and known angle, th2.

#     Parameters
#     ----------
#     th2 : float
#         the input angle at the actuating joint of the 4 bar linkage, aka angle DAB
#     a,b,c,d: floats
#         link lengths, defined in a clockwise manner from point A.
    
#     Returns
#     -------
#     ABC,BCD,CDA: floats
#         The remaining angles in the 4 bar linkage
    
#     """
#     # print('th2: ',m.degrees(th2),'a: ',a,'b: ',b,'c: ',c,'d: ',d)    
#     x_b = a*np.cos(th2)
#     y_b = a*np.sin(th2)
    
#     #define diagnonal f
#     f = np.sqrt((d-x_b)**2 +y_b**2)
#     beta = np.arccos((f**2+c**2-b**2)/(2*f*c))
#     gamma = np.arctan2(y_b,d-x_b)
    
#     th4 = np.pi - gamma - beta
    
#     x_c = c*np.cos(th4)+d
#     y_c = c*np.sin(th4)
    
#     th3 = np.arctan2((y_c-y_b),(x_c-x_b))
    
    
#     ## Calculate remaining internal angles of linkage
#     ABC = np.pi-th2 + th3
#     BCD  = th4-th3
#     CDA = np.pi*2 - th2 - ABC - BCD
                    
#     return ABC,BCD,CDA
    