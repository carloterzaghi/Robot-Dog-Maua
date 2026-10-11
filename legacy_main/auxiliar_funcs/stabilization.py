"""
auxiliar_funcs/stabilization.py — Shim de backward-compatibility.

A implementação real foi movida para motion/stabilization.py.
Este arquivo re-exporta todas as funções e constantes públicas
para que os scripts em test/ continuem funcionando sem modificação.
"""

# Re-exporta tudo do novo módulo
from motion.stabilization import (  # noqa: F401
    _KalmanFilter,
    _read_word,
    stabilize,
    stabilize_angular,
    stabilize_full_walking,
    MPU6050_ADDR,
    PWR_MGMT_1,
    ACCEL_XOUT_H,
    GYRO_XOUT_H,
    ANG_DIR_MIN, ANG_DIR_MAX,
    ANG_ESQ_MIN, ANG_ESQ_MAX,
    ANG_DIR_CENTER, ANG_ESQ_CENTER,
    ANG_DIR_RANGE, ANG_ESQ_RANGE,
    X_NOMINAL, Z_NOMINAL, Z_PITCH_RANGE,
    ROLL_MAX_DEG, PITCH_MAX_DEG,
    LOOP_DELAY,
)
