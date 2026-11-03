# 🎛️ Mechanical & Software Servo Calibration Guide

> **Relevant Modules:** [`Config.py`](../main/Config.py), [`HardwareInterface.py`](../main/HardwareInterface.py), [`Testes Interface/teste_servos.py`](../Testes%20Interface/teste_servos.py)

---

## 1. Why Calibration is Mandatory

Commercial standard servomotors utilize 25-tooth output splines (25T). Each individual tooth represents a discrete mechanical rotation of:
$$\frac{360^\circ}{25} = 14.4^\circ$$

When mounting the aluminum horn onto the servo shaft, **it is mechanically impossible to achieve an exact $90.0^\circ$ angle** through spline meshing alone. There will always be a residual assembly error between $\pm 0.5^\circ$ and $\pm 7.0^\circ$.

If left uncompensated in software:
* Legs will not be parallel, producing uneven weight distribution and premature motor burnout.
* Inverse kinematics equations will calculate trajectories based on Cartesian foot positions that do not match physical reality.
* The robot will pull to one side or lose balance during dynamic trotting.

---

## 2. The Nominal 90° Reference Pose

The calibration pipeline relies on a **fully orthogonal 90° calibration posture**:
- **Abduction Servo (Hip Roll):** Leg hangs strictly vertical relative to the chassis (zero lateral abduction tilt).
- **Femur Servo (Hip Pitch):** Upper leg segment is oriented perpendicular to the chassis base.
- **Tibia Servo (Knee Pitch):** Lower leg segment is oriented at exactly 90° relative to the femur.

In [`main/Config.py`](../main/Config.py), this reference posture is defined as:
```python
self.neutral_angle = np.array([
    [90.0, 90.0, 90.0, 90.0],  # Abduction (FR, FL, RR, RL)
    [90.0, 90.0, 90.0, 90.0],  # Femur
    [90.0, 90.0, 90.0, 90.0]   # Tibia
])
```

---

## 3. Step-by-Step Calibration Procedure

### Step 1: Send the 90° Neutral Signal (With Horns Detached!)
> ⚠️ **Warning:** Never force a servo horn or joint by hand while the motor is powered under active PWM holding torque.

1. Place the robot chassis on an elevated stand with all legs suspended in the air.
2. Remove the mounting screws securing the servo horns to all 12 servo shafts.
3. Start the system and press the **`X`** button on the gamepad (or execute [`Testes Interface/teste_servos.py`](../Testes%20Interface/teste_servos.py)).
4. All 12 servo output shafts will rotate to their nominal electronic 90° center position and lock in place.

### Step 2: Mechanical Horn Mounting
1. With the servos energized and held at electronic 90°, mount each horn onto the spline tooth that brings the joint closest to a true right angle ($90^\circ$).
2. Tighten the central horn retaining screws.

### Step 3: Measure Residual Angular Offsets
Using a digital protractor, square, or caliper, measure the angular deviation of each joint from true $90^\circ$:
* If the joint measures $92^\circ$, the residual error is $+2.0^\circ$.
* If the joint measures $87^\circ$, the residual error is $-3.0^\circ$.

### Step 4: Populate the Offset Matrix in `Config.py`
Insert your measured values directly into the `physical_calibration_offsets_unoriented` matrix in [`main/Config.py`](../main/Config.py):

```python
# Format: 
# Row 0: Abduction | Row 1: Femur | Row 2: Tibia
# Columns: [0]=FR, [1]=FL, [2]=RR, [3]=RL

self.physical_calibration_offsets_unoriented = np.array([
    [-5.50,  2.00,  6.00,  4.00],   # Abduction offsets
    [ 1.00,  1.50, -1.00, -1.00],   # Femur offsets
    [ 0.83,  2.83,  4.83,  0.00]    # Tibia offsets
])
```

---

## 4. Servo Orientation & Symmetry Inversion (`servo_orientation_adjusts`)

Because left-side legs are physically mirrored relative to right-side legs, identical positive servo rotations would cause one side to swing forward and the opposite side to swing backward.

To unify the kinematic coordinate frame, software applies an orientation mask:
```python
self.servo_orientation_adjusts = np.array([
    [0, 180, 180,   0],  # Abduction
    [0, 180,   0, 180],  # Femur
    [0, 180,   0, 180]   # Tibia
])
```

In `Config.py`, orientation directions are computed automatically:
```python
direction = np.where(self.servo_orientation_adjusts == 0, 1, -1)
self.physical_calibration_offsets_oriented = (
    self.physical_calibration_offsets_unoriented * direction
)
```

During hardware dispatch in [`main/HardwareInterface.py`](../main/HardwareInterface.py):
$$\theta_{\text{hardware}} = \text{clamp}(\theta_{\text{kinematic}} - \text{offset}_{\text{oriented}}, 0^\circ, 180^\circ)$$

---

## 5. Dynamic Pose Verification

After completing calibration, test the smooth posture transitions:
1. Press **`A`** on the gamepad to toggle between `Lay` and `Stand`. The robot should stand up smoothly and level, without gear clicks or servo chatter.
2. Verify that all 4 rubber feet make contact with a flat floor at the exact same instant. If one foot hangs off the ground or bears disproportionate weight, fine-tune the corresponding tibia/femur offset in `Config.py` in $0.5^\circ$ increments.
