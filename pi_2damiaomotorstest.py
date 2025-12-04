import time
from DM_CAN_Library import *
import serial

# =========================
# === MOTOR CONFIG ========
# =========================
serial_port = "/dev/ttyACM0"
baud_rate = 921600
MotorType = DM_Motor_Type.DM6006

# Motor CAN IDs
motor_ids = [0x01, 0x02]
feedback_ids = [0x11, 0x12]

MAX_SPEED = 6.0    # rad/s

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
print("Both motors enabled. Running test sequence...")

# =========================
# === MAIN LOOP ===========
# =========================
try:
    # Run both motors forward at half speed for 3 seconds
    print("Both motors forward at 3.0 rad/s...")
    MotorControl.control_Vel(Motors[0], 3.0)
    MotorControl.control_Vel(Motors[1], 3.0)
    time.sleep(3)
    
    # Stop both motors for 1 second
    print("Both motors stopped...")
    MotorControl.control_Vel(Motors[0], 0.0)
    MotorControl.control_Vel(Motors[1], 0.0)
    time.sleep(1)
    
    # Run both motors backward at half speed for 3 seconds
    print("Both motors backward at -3.0 rad/s...")
    MotorControl.control_Vel(Motors[0], -3.0)
    MotorControl.control_Vel(Motors[1], -3.0)
    time.sleep(3)
    
    # Stop both motors
    print("Both motors stopped...")
    MotorControl.control_Vel(Motors[0], 0.0)
    MotorControl.control_Vel(Motors[1], 0.0)
    time.sleep(1)
    

    
    # Stop both motors
    print("Both motors stopped...")
    MotorControl.control_Vel(Motors[0], 0.0)
    MotorControl.control_Vel(Motors[1], 0.0)
    time.sleep(1)
    
    print("Test sequence complete.")

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    # Stop and disable all motors
    for motor in Motors:
        MotorControl.control_Vel(motor, 0)
        MotorControl.disable(motor)
    serial_device.close()
    print("All motors disabled, serial closed.")