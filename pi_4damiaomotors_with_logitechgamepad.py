import time
from DM_CAN_Library import *
import serial
from inputs import get_gamepad

# =========================
# === MOTOR CONFIG ========
# =========================

serial_port = "/dev/ttyACM0"
baud_rate = 921600

MotorType = DM_Motor_Type.DM6006

# Motor CAN IDs
motor_ids = [0x01, 0x02, 0x03, 0x04]
feedback_ids = [0x11, 0x12, 0x13, 0x14]

MAX_SPEED = 6.0    # rad/s
DEADZONE_RY_LOW = -300
DEADZONE_RY_HIGH = 0
DEADZONE_LX_LOW = -300
DEADZONE_LX_HIGH = 400

# =========================
# === INITIALIZE SERIAL & MOTORS ===
# =========================

serial_device = serial.Serial(serial_port, baud_rate, timeout=0.5)

# Create motor objects
Motors = []
for m_id, f_id in zip(motor_ids, feedback_ids):
    motor = Motor(MotorType, m_id, f_id)
    Motors.append(motor)

# Create MotorControl object
MotorControl = MotorControl(serial_device)
for motor in Motors:
    MotorControl.addMotor(motor)
    MotorControl.switchControlMode(motor, Control_Type.VEL)
    MotorControl.enable(motor)

time.sleep(0.2)
print("All 4 motors enabled. Listening to joystick inputs...")

# =========================
# === MAIN LOOP ===========
# =========================

try:
    while True:
        events = get_gamepad()
        for event in events:
            
            # --- Forward/Backward with Right Joystick Y-axis ---
            if event.code == "ABS_RY":
                ry = event.state

                # Apply deadzone
                if DEADZONE_RY_LOW <= ry <= DEADZONE_RY_HIGH:
                    speed_fb = 0.0
                elif ry < DEADZONE_RY_LOW:
                    speed_fb = (ry / -32800.0) * MAX_SPEED
                else:
                    speed_fb = -(ry / 32800.0) * MAX_SPEED

                # Apply speed to all 4 motors (forward/backward)
                for idx, motor in enumerate(Motors):
                    if idx >= 2:  # Motors 3 and 4 are inverted
                        MotorControl.control_Vel(motor, -speed_fb)
                    else:
                        MotorControl.control_Vel(motor, speed_fb)

            # --- Left/Right with Left Joystick X-axis ---
            if event.code == "ABS_X":
                lx = event.state

                # Deadzone for small X values
                if DEADZONE_LX_LOW <= lx <= DEADZONE_LX_HIGH:
                    speed_lr = 0.0
                    MotorControl.control_Vel(Motors[0], 0.0)
                    MotorControl.control_Vel(Motors[1], 0.0)
                    MotorControl.control_Vel(Motors[2], 0.0)
                    MotorControl.control_Vel(Motors[3], 0.0)
                elif lx < DEADZONE_LX_LOW:
                    # Move left: Motors 1 & 2 backward, Motors 3 & 4 forward
                    speed_lr = (lx / -32800.0) * MAX_SPEED
                    MotorControl.control_Vel(Motors[0], -speed_lr)
                    MotorControl.control_Vel(Motors[1], -speed_lr)
                    MotorControl.control_Vel(Motors[2], -speed_lr)
                    MotorControl.control_Vel(Motors[3], -speed_lr)
                else:  # lx > DEADZONE_LX_HIGH
                    # Move right: Motors 1 & 2 forward, Motors 3 & 4 backward
                    speed_lr = (lx / 32800.0) * MAX_SPEED
                    MotorControl.control_Vel(Motors[0], speed_lr)
                    MotorControl.control_Vel(Motors[1], speed_lr)
                    MotorControl.control_Vel(Motors[2], speed_lr)
                    MotorControl.control_Vel(Motors[3], speed_lr)

                print(f"LX: {lx:6d}  ? LR Speed: {speed_lr:.2f} rad/s")

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    # Stop and disable all motors
    for motor in Motors:
        MotorControl.control_Vel(motor, 0)
        MotorControl.disable(motor)
    serial_device.close()
    print("All motors disabled, serial closed.")
