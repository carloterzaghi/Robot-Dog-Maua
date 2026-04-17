import time
import math
from smbus2 import SMBus
from adafruit_servokit import ServoKit # type: ignore

# ── Configurações do MPU6050 (GY-521) ────────────────────────────────────────
MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B

# ── Configurações do servo MG996R via PCA9685 ────────────────────────────────
SERVO_CHANNEL = 0   # canal do PCA9685


def read_word(bus: SMBus, addr: int, reg: int) -> int:
    """Lê dois bytes consecutivos e retorna valor com sinal (complemento de 2)."""
    high  = bus.read_byte_data(addr, reg)
    low   = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    if value >= 0x8000:
        value -= 65536
    return value


def compute_roll(ax: float, ay: float, az: float) -> float:
    """
    Calcula o ângulo de rolagem (roll) a partir do acelerômetro.
    Inclinando o sensor para a esquerda/direita → −90° a +90°.
    """
    return math.degrees(math.atan2(ay, math.sqrt(ax ** 2 + az ** 2)))


def main() -> None:
    # ── Inicializa PCA9685 / ServoKit ─────────────────────────────────────────
    print("Iniciando conexão com o PCA9685...")
    kit = ServoKit(channels=16)

    # ── Inicializa MPU6050 ────────────────────────────────────────────────────
    with SMBus(1) as bus:
        bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)  # sai do modo sleep
        print("MPU6050 e servo MG996R iniciados!")
        print("Incline o sensor para mover o servo. Ctrl+C para encerrar.\n")

        try:
            while True:
                # Leitura do acelerômetro (em g)
                ax = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H)     / 16384.0
                ay = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 2) / 16384.0
                az = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 4) / 16384.0

                # Ângulo de inclinação lateral (roll): −90° a +90°
                roll = compute_roll(ax, ay, az)

                # Mapeia −90°…+90° → 0°…180° e clipa para segurança
                servo_angle = max(0, min(180, int(roll + 90.0)))

                kit.servo[SERVO_CHANNEL].angle = servo_angle

                print(
                    f"Acel (g): X={ax:+.2f}  Y={ay:+.2f}  Z={az:+.2f} | "
                    f"Roll: {roll:+6.1f}° | Servo: {servo_angle}°"
                )

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\nEncerrando.")


if __name__ == "__main__":
    main()
