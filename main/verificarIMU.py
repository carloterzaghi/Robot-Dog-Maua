
from smbus2 import SMBus

MPU_ADDR = 0x68
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C

with SMBus(1) as bus:
    gyro_reg = bus.read_byte_data(MPU_ADDR, GYRO_CONFIG)
    accel_reg = bus.read_byte_data(MPU_ADDR, ACCEL_CONFIG)

    # Extrai os bits 4 e 3
    gyro_fs = (gyro_reg >> 3) & 0b11
    accel_fs = (accel_reg >> 3) & 0b11

    gyro_scales = [250, 500, 1000, 2000]
    gyro_sensitivity = [131.0, 65.5, 32.8, 16.4]

    accel_scales = [2, 4, 8, 16]

    print(f"GYRO_CONFIG: 0x{gyro_reg:02X}")
    print(f"Bits FS_SEL: {gyro_fs:02b}")
    print(f"Escala giroscópio: ±{gyro_scales[gyro_fs]}°/s")
    print(f"Sensibilidade: {gyro_sensitivity[gyro_fs]} LSB/(°/s)")

    print()
    print(f"ACCEL_CONFIG: 0x{accel_reg:02X}")
    print(f"Escala acelerômetro: ±{accel_scales[accel_fs]}g")
