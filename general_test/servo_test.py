import time
from adafruit_servokit import ServoKit

# Inicializa a placa especificando que ela possui 16 canais [cite: 542, 543]
kit = ServoKit(channels=16) # [cite: 544, 545]

canal0 = [0, 1, 2, 3]
canal1 = [4, 5, 6, 7]
canal2 = [8, 9, 10, 11]
canal3 = [12, 13, 14, 15]

canais = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
delay = 2

n = ""
while n != "6":
    print("\n" + "="*40)
    print("Escolha o canal para testar os servos:")
    print("0: Canal 0 (servos 0-3)")
    print("1: Canal 1 (servos 4-7)")
    print("2: Canal 2 (servos 8-11)")
    print("3: Canal 3 (servos 12-15)")
    print("4: Testa todos os canais (0-15)")
    print("5: Testar todos simultaneamente (0-15)")
    print("6: Finalizar teste")
    n = input("Digite a opção desejada: ").strip().lower()
    if n not in ['0', '1', '2', '3', '4', '5', '6']:
        print("Opção inválida. Tente novamente.")
        continue
    elif n == "6":
        print("Encerrando teste...")
    elif n == "5":
        print("--- Testando todos os canais simultaneamente ---")
        for i in canais:
            print(f"[Canal {i}] Movendo para 0 graus...")
            kit.servo[i].angle = 0
        time.sleep(delay)

        for i in canais:
            print(f"[Canal {i}] Movendo para 90 graus...")
            kit.servo[i].angle = 90

        for i in canais:
            print(f"[Canal {i}] Movendo para 180 graus...")
            kit.servo[i].angle = 180
        time.sleep(delay)
    else:
        n = int(n)
        d = {0: canal0, 1: canal1, 2: canal2, 3: canal3, 4: canais}
        print(f"--- Servos canal: {d[n]} ---")
        for i in d[n]:
            print(f"[Canal {i}] Movendo para 0 graus...")
            kit.servo[i].angle = 0
            time.sleep(delay)

            print(f"[Canal {i}] Movendo para 90 graus...")
            kit.servo[i].angle = 90
            time.sleep(delay)

print("Teste finalizado!")