"""Leitura do MPU6050 para MauAU: roll/pitch relativos e quaternion [w,x,y,z].

Montagem assumida: sensor +X frente, +Y baixo, +Z esquerda.
Yaw não é estimado. Execute calibrate() imóvel antes de controlar servos.
"""
import math
import time
import numpy as np
from smbus2 import SMBus


class KalmanFilter:
    def __init__(self):
        self.Q_angle = 0.001
        self.Q_bias = 0.003
        self.R_measure = 0.10
        self.reset()

    def reset(self):
        self.angle = 0.0
        self.bias = 0.0
        self.P = [[0.0, 0.0], [0.0, 0.0]]

    def get_angle(self, new_angle, new_rate, dt):
        dt = max(0.0, min(float(dt), 0.2))
        rate = new_rate - self.bias
        self.angle += dt * rate
        p = self.P
        p[0][0] += dt * (dt * p[1][1] - p[0][1] - p[1][0] + self.Q_angle)
        p[0][1] -= dt * p[1][1]
        p[1][0] -= dt * p[1][1]
        p[1][1] += self.Q_bias * dt
        s = p[0][0] + self.R_measure
        k0, k1 = p[0][0] / s, p[1][0] / s
        residual = new_angle - self.angle
        self.angle += k0 * residual
        self.bias += k1 * residual
        p00, p01 = p[0][0], p[0][1]
        p[0][0] -= k0 * p00
        p[0][1] -= k0 * p01
        p[1][0] -= k1 * p00
        p[1][1] -= k1 * p01
        return self.angle


class IMU:
    PWR_MGMT_1 = 0x6B
    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B
    ACCEL_XOUT_H = 0x3B

    def __init__(self, bus_number=1, address=0x68):
        self.MPU_ADDR = address
        self.bus = SMBus(bus_number)
        try:
            self.bus.write_byte_data(address, self.PWR_MGMT_1, 0x00)
            time.sleep(0.1)
            self.bus.write_byte_data(address, self.ACCEL_CONFIG, 0x00)  # +/-2 g
            self.bus.write_byte_data(address, self.GYRO_CONFIG, 0x00)   # +/-250 deg/s
            accel_cfg = self.bus.read_byte_data(address, self.ACCEL_CONFIG)
            gyro_cfg = self.bus.read_byte_data(address, self.GYRO_CONFIG)
            if ((accel_cfg >> 3) & 3) != 0 or ((gyro_cfg >> 3) & 3) != 0:
                raise RuntimeError('Escala MPU6050 incorreta')
            self.kalman_roll = KalmanFilter()
            self.kalman_pitch = KalmanFilter()
            self.roll_offset = 0.0
            self.pitch_offset = 0.0
            self.gyro_x_offset = 0.0
            self.gyro_z_offset = 0.0
            self.calibrated = False
            self.last_time = time.monotonic()
        except Exception:
            self.bus.close()
            raise

    def _apply_deadband(self, value, threshold=0.5):
        return 0.0 if abs(value) <= threshold else value

    @staticmethod
    def _signed16(high, low):
        value = (high << 8) | low
        return value - 65536 if value >= 32768 else value

    def read_raw(self):
        """Retorna (ax,ay,az) em g e (gx,gz) em graus/s."""
        data = self.bus.read_i2c_block_data(self.MPU_ADDR, self.ACCEL_XOUT_H, 14)
        ax = self._signed16(data[0], data[1]) / 16384.0
        ay = self._signed16(data[2], data[3]) / 16384.0
        az = self._signed16(data[4], data[5]) / 16384.0
        gx = self._signed16(data[8], data[9]) / 131.0
        gz = self._signed16(data[12], data[13]) / 131.0
        return ax, ay, az, gx, gz

    @staticmethod
    def _acc_angles(ax, ay, az):
        return (math.degrees(math.atan2(-az, -ay)),
                math.degrees(math.atan2(ax, -ay)))

    def calibrate(self, samples=300, interval=0.01):
        """Zera orientação na postura atual e estima bias gyro; mantenha imóvel."""
        if samples < 1:
            raise ValueError('samples deve ser >= 1')
        self.calibrated = False
        print('Calibrando IMU: mantenha o corpo nivelado e imóvel...')
        rolls, pitches, gxs, gzs = [], [], [], []
        for _ in range(samples):
            ax, ay, az, gx, gz = self.read_raw()
            r, p = self._acc_angles(ax, ay, az)
            rolls.append(r)
            pitches.append(p)
            gxs.append(gx)
            gzs.append(gz)
            time.sleep(interval)
        self.roll_offset = float(np.mean(rolls))
        self.pitch_offset = float(np.mean(pitches))
        self.gyro_x_offset = float(np.mean(gxs))
        self.gyro_z_offset = float(np.mean(gzs))
        self.kalman_roll.reset()
        self.kalman_pitch.reset()
        self.last_time = time.monotonic()
        self.calibrated = True
        print(f'Offsets: Roll={self.roll_offset:+.2f}°, Pitch={self.pitch_offset:+.2f}°')
        print(f'Gyro offsets: GX={self.gyro_x_offset:+.3f}°/s, GZ={self.gyro_z_offset:+.3f}°/s')
        return self.roll_offset, self.pitch_offset

    def read_angles(self):
        ax, ay, az, gx, gz = self.read_raw()

        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now

        # Ângulos do acelerômetro
        roll_acc = math.degrees(math.atan2(-az, -ay))
        pitch_acc = math.degrees(math.atan2(ax, -ay))

        # Correção dos offsets
        roll_acc_corr = roll_acc - self.roll_offset
        pitch_acc_corr = pitch_acc - self.pitch_offset

        # Correção do giroscópio
        gx_corr = gx - self.gyro_x_offset
        gz_corr = gz - self.gyro_z_offset

        # Filtro de Kalman
        roll = self.kalman_roll.get_angle(roll_acc_corr, -gx_corr, dt)
        pitch = self.kalman_pitch.get_angle(pitch_acc_corr, -gz_corr, dt)

        # Deadband: valores pequenos viram zero
        roll = self._apply_deadband(roll, threshold=0.5)
        pitch = self._apply_deadband(pitch, threshold=0.5)

        return roll, pitch

    # def read_orientation(self):
    #     """Quaternion normalizado [w,x,y,z] (Euler yaw=0)."""
    #     roll_deg, pitch_deg = self.read_angles()
    #     roll = math.radians(roll_deg)
    #     pitch = math.radians(pitch_deg)
    #     cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    #     cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    #     q = np.array([cr * cp, sr * cp, cr * sp, -sr * sp], dtype=float)
    #     return q / np.linalg.norm(q)

    def read_orientation(self):
        roll_deg, pitch_deg = self.read_angles()

        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)

        cr = math.cos(roll / 2)
        sr = math.sin(roll / 2)
        cp = math.cos(pitch / 2)
        sp = math.sin(pitch / 2)

        q = np.array([cr * cp, sr * cp, cr * sp, -sr * sp], dtype=float)
        return q / np.linalg.norm(q)

    def close(self):
        self.bus.close()
