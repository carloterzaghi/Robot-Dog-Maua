import numpy as np
import math as m

# TODO: put these somewhere else
class PWMParams:
    def __init__(self):
        # FR FL RR RL
        # ANGULAR
        # FEMUR
        # TIBIA
        self.pins = np.array([[1, 6, 14, 9], 
                              [2, 7, 13, 8], 
                              [0, 5, 15, 10]])
        #self.range = 4000 acho q n precisa
        #self.freq = 250 acho q n precisa


class ServoParams:
    def __init__(self):
        #self.neutral_position_pwm = 1500  # Middle position
        self.micros_per_rad = MICROS_PER_RAD  # Must be calibrated

        # The neutral angle of the joint relative to the modeled zero-angle in degrees, for each joint
        self.neutral_angle_degrees = np.array(
            [[  90.,  90.,  90.,  90.],
            [ 90., 90., 90., 90.],
            [-90.,-90.,-90.,-90.]]
        )

        self.servo_multipliers = np.array(
            [[1, 1, 1, 1], 
             [-1, 1, -1, 1], 
             [1, -1, 1, -1]]
        )

        # FR FL RR RL
        # ANGULAR
        # FEMUR
        # TIBIA

        self.servo_multipliers = np.array(
            [[0, -180, 0, -180, ], 
             [0, -180, 0, -180], 
             [0, -180, 0, -180]]
        )

    # @property
    # def neutral_angles(self):
    #     return self.neutral_angle_degrees * np.pi / 180.0  # Convert to radians


class Configuration:
    def __init__(self):

        # FR FL RR RL
        # ANGULAR
        # FEMUR
        # TIBIA
        self.pins = np.array([[1, 6, 14, 9], 
                              [2, 7, 13, 8], 
                              [0, 5, 15, 10]])
        #self.range = 4000 acho q n precisa
        #self.freq = 250 acho q n precisa


        self.servo_multipliers = np.array(
            [[0, -180, 0, -180, ], 
             [0, -180, 0, -180], 
             [0, -180, 0, -180]]
        )

        #################### COMMANDS ####################
        self.max_x_velocity = 0.4
        self.max_y_velocity = 0.3
        self.max_yaw_rate = 2.0
        self.max_pitch = 30.0 * np.pi / 180.0
        
        #################### MOVEMENT PARAMS ####################
        self.z_time_constant = 0.02
        self.z_speed = 0.03  # maximum speed [m/s]
        self.pitch_deadband = 0.02
        self.pitch_time_constant = 0.25
        self.max_pitch_rate = 0.15
        self.roll_speed = 0.16  # maximum roll rate [rad/s]
        self.yaw_time_constant = 0.3
        self.max_stance_yaw = 1.2
        self.max_stance_yaw_rate = 2.0

        #################### STANCE #################### posições de pé
        self.delta_x = 0.126 #posição X do pé em relação a origem do robo quando fica de pé
        self.delta_y = 0.137 #posição Y do pé em relação a origem do robo quando fica de pé
        self.x_shift = 0.0 
        self.default_z_ref = -0.22 #ALTURA PADRÃO DO ROBO EM PÉ [m]

        #################### SWING ######################
        self.z_coeffs = None
        self.z_clearance = 0.07 #ALTURA MAXIMA DE SWING DA PATA QUANDO EM MOVMENTO
        self.alpha = (
            0.5  # Ratio between touchdown distance and total horizontal stance movement
        )
        self.beta = (
            0.5  # Ratio between touchdown distance and total horizontal stance movement
        )

        #################### GAIT #######################
        self.dt = 0.01
        self.num_phases = 4
        self.contact_phases = np.array(
            [[1, 1, 1, 0], 
             [1, 0, 1, 1], 
             [1, 0, 1, 1], 
             [1, 1, 1, 0]]
        )
        self.overlap_time = (
            0.10  # duration of the phase where all four feet are on the ground
        )
        self.swing_time = (
            0.15  # duration of the phase when only two feet are on the ground
        )

        ######################## GEOMETRY ######################
        self.LEG_FRONT_X = 0.1145  # front-back distance from center line to front leg axis
        self.LEG_REAR_X = 0.1375  # front-back distance from center line to rear leg axis
        self.LEG_LR = 0.097  # left-right distance from center line to leg origin plane
        self.LEG_L1 = 0.130
        self.LEG_L2 = 0.120
        self.ABDUCTION_OFFSET = 0.04  # distance from abduction axis to leg
        self.FOOT_RADIUS = 0.014


        self.a = 0.035 #mm
        self.b = 0.03253 #mm
        self.c = 0.035 #mm
        self.d = 0.03253  #mm

        self.e = 0.0495 #mm 11

        self.f = self.LEG_L2 #mm  
        self.g = 0.035 #mm 11 
        self.h = 0.035 #mm 11
        self.i =  self.LEG_L2 #mm        

        
        self.gamma = m.atan(23/23)
        self.EDC = m.acos(( self.c**2+ self.h**2- self.e**2)/(2* self.c* self.h))
        self.phi = m.pi/2

        self.LEG_ORIGINS = np.array(
            [
                [self.LEG_FRONT_X, self.LEG_FRONT_X, -self.LEG_REAR_X, -self.LEG_REAR_X],
                [-self.LEG_LR, self.LEG_LR, -self.LEG_LR, self.LEG_LR],
                [0, 0, 0, 0],
            ]
        )

        self.ABDUCTION_OFFSETS = np.array(
            [
                -self.ABDUCTION_OFFSET,
                self.ABDUCTION_OFFSET,
                -self.ABDUCTION_OFFSET,
                self.ABDUCTION_OFFSET,
            ]
        )

    @property
    def default_stance(self):
        return np.array(
            [
                [
                    self.delta_x + self.x_shift,
                    self.delta_x + self.x_shift,
                    -self.delta_x + self.x_shift,
                    -self.delta_x + self.x_shift,
                ],
                [-self.delta_y, self.delta_y, -self.delta_y, self.delta_y],
                [0, 0, 0, 0],
            ]
        )

    ################## SWING ###########################
    @property
    def z_clearance(self):
        return self.__z_clearance

    @z_clearance.setter
    def z_clearance(self, z):
        self.__z_clearance = z


    ########################### GAIT ####################
    @property
    def overlap_ticks(self):
        return int(self.overlap_time / self.dt)

    @property
    def swing_ticks(self):
        return int(self.swing_time / self.dt)

    @property
    def stance_ticks(self):
        return 2 * self.overlap_ticks + self.swing_ticks

    @property
    def phase_ticks(self):
        return np.array(
            [self.overlap_ticks, self.swing_ticks, self.overlap_ticks, self.swing_ticks]
        )

    @property
    def phase_length(self):
        return 2 * self.overlap_ticks + 2 * self.swing_ticks

        
