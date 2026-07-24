from smbus2 import SMBus
import time
import math

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


class KalmanFilter:
    """
    Filtro de Kalman para estimativa de ângulo usando acelerômetro e giroscópio.
    Baseado na implementação clássica de Kristian Lauszus (TKJ Electronics).
    """

    def __init__(self):
        # Variância do ruído do processo (quanto confiamos no giroscópio)
        self.Q_angle = 0.001
        # Variância do ruído do viés do giroscópio
        self.Q_bias = 0.003
        # Variância do ruído de medição (quanto confiamos no acelerômetro)
        self.R_measure = 0.03

        self.angle = 0.0   # Ângulo estimado pelo filtro
        self.bias = 0.0    # Viés estimado do giroscópio

        # Matriz de covariância do erro (2x2)
        self.P = [[0.0, 0.0],
                  [0.0, 0.0]]

    def get_angle(self, new_angle, new_rate, dt):
        """
        Atualiza o filtro de Kalman e retorna o ângulo estimado.

        Args:
            new_angle (float): Ângulo medido pelo acelerômetro (graus)
            new_rate  (float): Taxa angular medida pelo giroscópio (°/s)
            dt        (float): Intervalo de tempo desde a última chamada (segundos)

        Returns:
            float: Ângulo filtrado estimado (graus)
        """
        # --- Etapa de Predição ---
        # Prediz o ângulo com base na taxa do giroscópio corrigida pelo viés
        rate = new_rate - self.bias
        self.angle += dt * rate

        # Atualiza a matriz de covariância
        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        # --- Etapa de Atualização (Correção) ---
        # Inovação: diferença entre medição e predição
        S = self.P[0][0] + self.R_measure

        # Ganho de Kalman
        K = [self.P[0][0] / S,
             self.P[1][0] / S]

        # Resíduo da medição
        y = new_angle - self.angle

        # Corrige o ângulo e o viés com base no ganho
        self.angle += K[0] * y
        self.bias  += K[1] * y

        # Atualiza a covariância do erro
        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]
        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp

        return self.angle


def main():
    # Instancia os filtros de Kalman para pitch (X) e roll (Y)
    kalman_pitch = KalmanFilter()
    kalman_roll  = KalmanFilter()

    with SMBus(1) as bus:
        # Acorda o MPU6050
        bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
        print("MPU6050 inicializado!\n")

        # Leitura inicial para inicializar os ângulos do filtro
        accel_x = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H) / 16384.0
        accel_y = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 2) / 16384.0
        accel_z = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 4) / 16384.0

        # Ângulo inicial calculado pelo acelerômetro
        pitch_acc = math.degrees(math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)))
        roll_acc  = math.degrees(math.atan2(-accel_x, accel_z))

        kalman_pitch.angle = pitch_acc
        kalman_roll.angle  = roll_acc

        timer = time.time()

        while True:
            # --- Leitura do acelerômetro ---
            accel_x = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H)     / 16384.0
            accel_y = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 2) / 16384.0
            accel_z = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 4) / 16384.0

            # --- Leitura do giroscópio ---
            gyro_x = read_word(bus, MPU6050_ADDR, GYRO_XOUT_H)     / 131.0
            gyro_y = read_word(bus, MPU6050_ADDR, GYRO_XOUT_H + 2) / 131.0
            gyro_z = read_word(bus, MPU6050_ADDR, GYRO_XOUT_H + 4) / 131.0

            # --- Intervalo de tempo ---
            now = time.time()
            dt  = now - timer
            timer = now

            # --- Ângulos pelo acelerômetro (referência) ---
            pitch_acc = math.degrees(math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)))
            roll_acc  = math.degrees(math.atan2(-accel_x, accel_z))

            # --- Aplicação do filtro de Kalman ---
            pitch_kalman = kalman_pitch.get_angle(pitch_acc, gyro_x, dt)
            roll_kalman  = kalman_roll.get_angle(roll_acc,  gyro_y, dt)

            # --- Saída no terminal ---
            print(f"Acel   (g)  : X={accel_x:+.3f}  Y={accel_y:+.3f}  Z={accel_z:+.3f}")
            print(f"Giro  (°/s) : X={gyro_x:+.2f}  Y={gyro_y:+.2f}  Z={gyro_z:+.2f}")
            print(f"Acel  (°)   : Pitch={pitch_acc:+.2f}  Roll={roll_acc:+.2f}")
            print(f"Kalman(°)   : Pitch={pitch_kalman:+.2f}  Roll={roll_kalman:+.2f}")
            print("-" * 55)

            time.sleep(0.05)  # ~20 Hz — ciclo mais rápido para o filtro ser eficaz


if __name__ == "__main__":
    main()