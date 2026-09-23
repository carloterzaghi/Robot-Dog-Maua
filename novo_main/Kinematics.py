import numpy as np
from numpy.linalg import inv, norm
import math as m
from Config import Configuration
from time import sleep

def point_to_rad(p1, p2): # converts 2D cartesian points to polar angles in range 0 - 2pi
    theta = m.atan2(p2, p1)
    theta = (theta + 2*m.pi) % (2*m.pi)
    return theta

def exibe_deg(ang, text, printa):
    if(printa): print(f"\n{text} = {ang:.4f} rad -> {ang*180/m.pi:.4f} deg")

def calculate_4_bar(th2 ,a,b,c,d):
    """Using 'Freudensteins method', it finds all the angles within a 4 bar linkage with vertices ABCD and known link lengths a,b,c,d
    defined clockwise from point A, and known angle, th2.

    Parameters
    ----------
    th2 : float
        the input angle at the actuating joint of the 4 bar linkage, aka angle DAB
    a,b,c,d: floats
        link lengths, defined in a clockwise manner from point A.
    
    Returns
    -------
    ABC,BCD,CDA: floats
        The remaining angles in the 4 bar linkage
    
    """
    #print('th2: ',m.degrees(th2),'a: ',a,'b: ',b,'c: ',c,'d: ',d)    
    x_b = a*np.cos(th2)
    y_b = a*np.sin(th2)
    
    #define diagnonal f
    f = np.sqrt((d-x_b)**2 +y_b**2)
    beta = np.arccos((f**2+c**2-b**2)/(2*f*c))
    gamma = np.arctan2(y_b,d-x_b)
    
    th4 = np.pi - gamma - beta
    
    x_c = c*np.cos(th4)+d
    y_c = c*np.sin(th4)
    
    th3 = np.arctan2((y_c-y_b),(x_c-x_b))
    
    
    ## Calculate remaining internal angles of linkage
    ABC = np.pi-th2 + th3
    BCD  = th4-th3
    CDA = np.pi*2 - th2 - ABC - BCD
                    
    return ABC,BCD,CDA

# def lower_leg_angle_to_servo_angle(config, THETA2, THETA3):
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
#     GDE,DEF,EFG = calculate_4_bar(THETA3,config.i,config.h,config.f,config.g) #+ config.lower_leg_bend_angle
#     # Triangle section
#     CDH = 3/2*m.pi - THETA2 - GDE - config.EDC
#     CDA = CDH +config.gamma #input angle
#     # Second 4 bar linkage
#     DAB,ABC,BCD = calculate_4_bar(CDA ,config.d,config.a,config.b,config.c)
#     #Calculating Theta
#     THETA0 = DAB + config.gamma

#     return THETA0

def RotMatrix3D(rotation=[0,0,0],is_radians=True, order='xyz'):
    
    roll, pitch, yaw = rotation[0], rotation[1], rotation[2]

    # convert to radians is the input is in degrees
    if not is_radians: 
        roll = m.radians(roll)
        pitch = m.radians(pitch)
        yaw = m.radians(yaw)
    
    # rotation matrix about each axis
    rotX = np.matrix([[1, 0, 0], [0, m.cos(roll), -m.sin(roll)], [0, m.sin(roll), m.cos(roll)]])
    rotY = np.matrix([[m.cos(pitch), 0, m.sin(pitch)], [0, 1, 0], [-m.sin(pitch), 0, m.cos(pitch)]])
    rotZ = np.matrix([[m.cos(yaw), -m.sin(yaw), 0], [m.sin(yaw), m.cos(yaw), 0], [0, 0, 1]])
    
    # rotation matrix order (default: pitch -> roll -> yaw)
    if order == 'xyz': rotationMatrix = rotZ * rotY * rotX
    elif order == 'xzy': rotationMatrix = rotY * rotZ * rotX
    elif order == 'yxz': rotationMatrix = rotZ * rotX * rotY
    elif order == 'yzx': rotationMatrix = rotX * rotZ * rotY
    elif order == 'zxy': rotationMatrix = rotY * rotX * rotZ
    elif order == 'zyx': rotationMatrix = rotX * rotY * rotZ
    
    return rotationMatrix # roll pitch and yaw rotation 

#-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/

def impose_physical_limits(desired_joint_angles):
    ''' Takes desired upper and lower leg angles and clips them to be within the range of physical possiblity. 
    This is because some angles are not possible for the physical linkage. Processing is done in degrees.
        ----------
    desired_joint_angles : numpy array 3x4 of float angles (radians)
        Desired angles of all joints for all legs from inverse kinematics

    Returns
    -------
    possble_joint_angles: numpy array 3x4 of float angles (radians)
        The angles that will be attempted to be implemeneted, limited to a possible range
    '''
    possible_joint_angles = np.zeros((3,4))

    for i in range(4):
        hip,upper,lower = np.degrees(desired_joint_angles[:,i])

        hip   = np.clip(hip,-20,20)
        upper = np.clip(upper,5,180)

        if      0    <=  upper <     10  :
            lower = np.clip(lower, -20 , 40) 
        elif 10    <=  upper <     20  :
            lower = np.clip(lower, -40 , 40)
        elif 20    <=  upper <     30  :
            lower = np.clip(lower, -50 , 40) 
        elif 30    <=  upper <     40  :
            lower = np.clip(lower, -60 , 30) 
        elif 40    <=  upper <     50  :
            lower = np.clip(lower, -70 , 25)
        elif 50    <=  upper <     60  :
            lower = np.clip(lower, -70 , 20) 
        elif 60    <=  upper <     70  :
            lower = np.clip(lower, -70 , 0) 
        elif 70    <=  upper <     80  :
            lower = np.clip(lower, -70 , -10)
        elif 80    <=  upper <     90  :
            lower = np.clip(lower, -70 , -20) 
        elif 90    <=  upper <     100  :
            lower = np.clip(lower, -70 , -30) 
        elif 100    <=  upper <     110  :
            lower = np.clip(lower, -70 , -40)
        elif 110    <=  upper <     120  :
            lower = np.clip(lower, -70 , -60) 

        possible_joint_angles[:,i] =  hip,upper,lower

    return np.radians(possible_joint_angles)

def angle_corrector(angles=[0,0,0]):

    angles[0] = 2*m.pi - angles[0] + m.pi/2
    angles[2] = angles[1] + angles[2] - 5/4*m.pi #theta3 offset
    angles[1] = angles[1] - m.pi/2 #theta2 offset
    

    for index, theta in enumerate(angles):
        if theta > 2*m.pi: angles[index] = np.mod(theta,2*m.pi)
        if theta > m.pi: angles[index] = -(2*m.pi - theta)
    return angles

def Calculate_IK(r_body_foot, leg_index, config):
    is_right = 0

    x=r_body_foot[0]
    y=r_body_foot[1]
    z=r_body_foot[2]

    if leg_index == 0 or leg_index == 2:
        
        is_right = 1

    if is_right: 
        y = -y

    # print(f"x = {x:.4f}")
    # print(f"y = {y:.4f}")
    # print(f"z = {z}")

    r_body_foot = np.array([x,y,z])

    # length of vector projected on the YZ plane. equiv. to len_A = sqrt(y**2 + z**2)
    len_A = norm([0,y,z])   
    # a_1 : angle from the positive y-axis to the end-effector (0 <= a_1 < 2pi)
    # a_2 : angle bewtween len_A and leg's projection line on YZ plane
    # a_3 : angle between link1 and length len_A
    a_1 = point_to_rad(y,z)                     
    a_2 = m.asin(m.sin(config.phi)*config.ABDUCTION_OFFSET/len_A)
    a_3 = m.pi - a_2 - config.phi               

    # angle of link1 about the x-axis 
    if is_right: theta_1 = (a_1 + a_3)
    else: 
        theta_1 = a_1 + a_3
    if theta_1 >= 2*m.pi: theta_1 = np.mod(theta_1,2*m.pi)

    #Translate frame to the frame of the leg
    offset = np.array([0.0,config.ABDUCTION_OFFSET*m.cos(theta_1),config.ABDUCTION_OFFSET*m.sin(theta_1)])
    translated_frame = r_body_foot - offset

    R2 = theta_1 + config.phi - m.pi/2

    # create rotation matrix to work on a new 2D plane (XZ_)
    rot_mtx = RotMatrix3D([-R2,0,0],is_radians=True)
    j4_2_vec_ = rot_mtx * (np.reshape(translated_frame,[3,1]))
    j4_2_vec_ = np.ravel(j4_2_vec_)

    # xyz in the rotated coordinate system + offset due to link_1 removed
    x_, y_, z_ = j4_2_vec_[0], j4_2_vec_[1], j4_2_vec_[2]
    
    len_B = norm([x_, 0, z_])

    if len_B >= (0.230): 
        len_B = (0.230)
        #rospy.logwarn('target coordinate: [%f %f %f] too far away', x, y, z)

    # Após o clamp superior (linha 226), adicionar:
    # if len_B < abs(config.LEG_L1 - config.LEG_L2):
    #     len_B = abs(config.LEG_L1 - config.LEG_L2)

    # b_1 : angle between +ve x-axis and len_B (0 <= b_1 < 2pi)
    # b_2 : angle between len_B and link_2
    # b_3 : angle between link_2 and link_3
    b_1 = point_to_rad(x_, z_)  
    # b_2 = m.acos((config.LEG_L2**2 + len_B**2 - config.LEG_L1**2) / (2 * config.LEG_L2 * len_B)) 
    # b_3 = m.acos((config.LEG_L2**2 + config.LEG_L1**2 - len_B**2) / (2 * config.LEG_L2 * config.LEG_L1))  

    # Depois (linhas 233-234) — clamp para evitar math domain error
    
    arg_b2 = (config.LEG_L2**2 + len_B**2 - config.LEG_L1**2) / (2 * config.LEG_L2 * len_B)
    b_2 = m.acos(max(-1.0, min(1.0, arg_b2)))

    arg_b3 = (config.LEG_L2**2 + config.LEG_L1**2 - len_B**2) / (2 * config.LEG_L2 * config.LEG_L1)
    b_3 = m.acos(max(-1.0, min(1.0, arg_b3)))


    # print(f"b_1 = {b_1}")
    # print(f"b_2 = {b_2}")
    # print(f"b_3 = {b_3}")

    theta_2 = b_1 - b_2
    theta_3 = m.pi - b_3

    # print(f"theta_1 = {theta_1}")
    # print(f"theta_2 = {theta_2}")
    # print(f"theta_3 = {theta_3}")

    unrevised_angles = [theta_1,theta_2,theta_3]
    revised_angles = np.rad2deg(angle_corrector(unrevised_angles))


    # print("revised angels:")

    # print(np.matrix(revised_angles))

    return[revised_angles[0],revised_angles[1],revised_angles[2]]



def phisicalLimits(angulos = [0,0,0]):
    return

def setLegPosition(foot_position=[0,0,0], config = None): #substituida por four_legs_inverse_kinematics
    angulos = Calculate_IK(foot_position, config)
    # modify angles to match robot's configuration (i.e., adding offsets)
    angulos = angle_corrector(angulos)
    ##angulos = phisicalLimits()
    return [angulos[0], angulos[1], angulos[2]]

def four_legs_inverse_kinematics(r_body_foot, config):
    """Find the joint angles for all twelve DOF correspoinding to the given matrix of body-relative foot positions.
    
    Parameters
    ----------
    r_body_foot : numpy array (3,4)
        Matrix of the body-frame foot positions. Each column corresponds to a separate foot.
    config : Config object
        Object of robot configuration parameters.
    
    Returns
    -------
    numpy array (3,4)
        Matrix of corresponding joint angles.
    """
    alpha = np.zeros((3, 4))
    for i in range(4):
        body_offset = config.LEG_ORIGINS[:, i]
        #print(np.matrix(r_body_foot[:, i] - body_offset))
        alpha[:, i] = Calculate_IK(
            r_body_foot[:, i] - body_offset, i, config
        )
    #print(" ")
    #print("Alpha:")
    #print(np.matrix(alpha))
    #print(" ")
    return alpha
    
#--------------------------------------------------------------< Comeco do Codigo >--------------------------------------------------------------#

config = Configuration()

foot_locations = (
            config.default_stance
            + np.array([0, 0, config.default_z_ref])[:, np.newaxis]
)


joint_angles = four_legs_inverse_kinematics(
            foot_locations, config
        )

#print(np.matrix(np.rad2deg(joint_angles)))




# print("X:")
# x = float(input())
# print("Y:")
# y = float(input())
# print("Z:")
# z = float(input())

# x = 0.0115 #olhando de lado = frente e tras
# y = -0.04 #olhando de lado = entrando e saindo (se olhar de cima, Y para a esquerda é + e para a direita é -)
# z = -0.22 #olhando de lado = cima e baixo



# posicao = [x,y,z]


# angulos = four_legs_inverse_kinematics(posicao, config)

# print(f"\nPontos: {posicao[0]}, {posicao[1]}, {posicao[2]}")
# exibe_deg(angulos[0],"theta_1 ajustado",1)

# exibe_deg(angulos[1],"theta_2 ajustado",1)

# exibe_deg(angulos[2],"theta_3 ajustado",1)
# print("\n")
