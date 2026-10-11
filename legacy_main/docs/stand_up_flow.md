# Fluxo: Robô Ficando de Pé

Documentação do passo a passo do código executado quando o robô sai da posição deitada (sleep) e fica de pé.

---

## Gatilho — Botão Y pressionado

[`input/gamepad.py:L287-L296`](../input/gamepad.py) — O evento de botão dispara `_toggle_stand_sleep()`.

---

## Passo 1 — Para a marcha e solta o estado

[`input/gamepad.py:L376-L381`](../input/gamepad.py)

```python
self._gait.stop()              # para threads de marcha se estiverem rodando
self.shared_state["speed"] = 0
threading.Thread(target=self._run_toggle, daemon=True).start()
```

---

## Passo 2 — `poses.stand(servo_mgr)`

[`input/gamepad.py:L388`](../input/gamepad.py) → [`motion/poses.py:L50-L62`](../motion/poses.py)

Executa a transição em **2 etapas** para não sobrecarregar os servos:

```python
# Etapa 1: fêmures e tíbias vão para os ângulos de POSE_STAND
servo_mgr.smooth_move(stand_struct, ...)   # sem angulares

# Etapa 2: angulares vão para a posição neutra
servo_mgr.smooth_move(stand_ang, ...)
```

Cada `smooth_move` interpola em **60 passos** com curva **Smootherstep** (C²) ao longo de ~1,2 s.  
Os **offsets de calibração** de `configs/calibration.json` são somados automaticamente.

> **Nota:** `POSE_STAND` (em `configs/robot_config.py:L135`) **não é a posição final de pé** — é apenas uma pose de transição para o robô sair do chão de forma segura.

### Ângulos de `POSE_STAND` (transição)

| Servo | Ângulo (°) |
|---|---|
| `frente_femur_dir` | 10.0 |
| `frente_tibia_dir` | 30.0 |
| `frente_femur_esq` | 165.0 |
| `frente_tibia_esq` | 155.0 |
| `tras_femur_dir` | 20.0 |
| `tras_tibia_dir` | 20.0 |
| `tras_femur_esq` | 170.0 |
| `tras_tibia_esq` | 150.0 |
| `frente_angular_dir` | 101.0 |
| `frente_angular_esq` | 110.0 |
| `tras_angular_dir` | 102.0 |
| `tras_angular_esq` | 100.0 |

---

## Passo 3 — `gait.start(shared_state)` → `ramp_to_start()`

[`input/gamepad.py:L389`](../input/gamepad.py) → [`motion/locomotion.py:L261`](../motion/locomotion.py) → [`core/leg.py:L123`](../core/leg.py)

Lança **4 threads paralelas** (uma por perna). Cada thread executa `ramp_to_start()`:

```python
z_apoio = -220  # de GAIT_PARAMS em robot_config.py

# Calcula ângulos via cinemática inversa
femur_alvo, tibia_alvo = ik_to_servo_angles(x=0, z=-220, mirror=...)

# Interpola suavemente em N_RAMP=40 passos (RAMP_DELAY=0.025s entre cada)
for i in range(40):
    servo_mgr.set_angle(femur,   femur_ramp[i])
    servo_mgr.set_angle(tibia,   tibia_ramp[i])
    servo_mgr.set_angle(angular, angular_ramp[i])
    time.sleep(0.025)
```

As 4 threads se **sincronizam em uma barreira** (`threading.Barrier`) antes de iniciar o loop de marcha.

### Ângulos reais de pé — resultado da IK (`x=0, z=-220mm`)

Estes são os ângulos **verdadeiros** quando o robô está de pé e pronto para andar:

| Servo | Ângulo (°) |
|---|---|
| `frente_femur_dir` | 150.4 |
| `frente_tibia_dir` | 64.1 |
| `frente_femur_esq` | 29.6 |
| `frente_tibia_esq` | 115.9 |
| `tras_femur_dir` | 150.4 |
| `tras_tibia_dir` | 64.1 |
| `tras_femur_esq` | 29.6 |
| `tras_tibia_esq` | 115.9 |
| `frente_angular_dir` | 101.0 (fixo, `ang_min`) |
| `frente_angular_esq` | 110.0 (fixo, `ang_min`) |
| `tras_angular_dir` | 102.0 (fixo, `ang_min`) |
| `tras_angular_esq` | 100.0 (fixo, `ang_min`) |

> Estes ângulos **não existem hardcoded** em nenhum lugar — são sempre calculados na hora pela IK a partir de `z_apoio = -220mm`.

---

## Fluxo Completo

```
[Botão Y]
    │
    ▼
_toggle_stand_sleep()            gamepad.py:L369
    │
    ▼
_run_toggle()  — nova thread     gamepad.py:L383
    │
    ├─► poses.stand(servo_mgr)               poses.py:L50
    │       │
    │       ├─► smooth_move(fêmures+tíbias → POSE_STAND)
    │       │       └─► 60 passos Smootherstep → srv.angle (PCA9685)
    │       │
    │       └─► smooth_move(angulares → POSE_STAND)
    │               └─► 60 passos Smootherstep → srv.angle (PCA9685)
    │
    └─► gait.start()                         locomotion.py:L251
            │
            └─► 4 threads paralelas → ramp_to_start()    leg.py:L123
                    │
                    ├─► ik_to_servo_angles(0, -220)       kinematics.py:L131
                    │       └─► femur ≈ 150°/29°, tibia ≈ 64°/116°
                    │
                    ├─► 40 passos Smootherstep → set_angle() → srv.angle
                    │
                    └─► barreira de sincronização → loop de marcha
```

---

## Arquivos envolvidos

| Arquivo | Papel |
|---|---|
| [`input/gamepad.py`](../input/gamepad.py) | Detecta o botão Y e dispara a sequência |
| [`motion/poses.py`](../motion/poses.py) | Executa a transição `stand()` (POSE_STAND) |
| [`core/servo_manager.py`](../core/servo_manager.py) | `smooth_move()` — interpola e escreve nos servos |
| [`motion/locomotion.py`](../motion/locomotion.py) | Lança as 4 threads de perna |
| [`core/leg.py`](../core/leg.py) | `ramp_to_start()` — rampa via IK até posição de marcha |
| [`core/kinematics.py`](../core/kinematics.py) | `ik_to_servo_angles()` — calcula ângulos reais |
| [`configs/robot_config.py`](../configs/robot_config.py) | `POSE_STAND`, `GAIT_PARAMS`, `N_RAMP`, `RAMP_DELAY` |
