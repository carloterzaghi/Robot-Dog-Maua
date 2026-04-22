from smbus2 import SMBus
import time

# Endereço padrão do MPU6050
MPU6050_ADDR = 0x68

# Registradores
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

def read_word(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) + low
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    return value

def main():
    with SMBus(1) as bus:
        # Acorda o MPU6050
        bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

        print("MPU6050 inicializado!\n")

        while True:
            # Leitura acelerômetro
            accel_x = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H) / 16384.0
            accel_y = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 2) / 16384.0
            accel_z = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 4) / 16384.0

            # Leitura giroscópio
            gyro_x = read_word(bus, MPU6050_ADDR, GYRO_XOUT_H) / 131.0
            gyro_y = read_word(bus, MPU6050_ADDR, GYRO_XOUT_H + 2) / 131.0
            gyro_z = read_word(bus, MPU6050_ADDR, GYRO_XOUT_H + 4) / 131.0

            print(f"Acel (g): X={accel_x:.2f} Y={accel_y:.2f} Z={accel_z:.2f}")
            print(f"Giro (°/s): X={gyro_x:.2f} Y={gyro_y:.2f} Z={gyro_z:.2f}")
            print("-" * 50)

            time.sleep(0.5)

if __name__ == "__main__":
    main()