import time
from DM_CAN_Library import *
import serial

# =========================
# === MOTOR CONFIG ========
# =========================
serial_port = "/dev/ttyACM0"
baud_rate = 921600
MotorType = DM_Motor_Type.DM6006

# Single Motor CAN IDs
motor_id = 0x01
feedback_id = 0x11

MAX_SPEED = 6.0    # rad/s

# =========================
# === INITIALIZE SERIAL & MOTOR ===
# =========================
serial_device = serial.Serial(serial_port, baud_rate, timeout=0.5)

# Create motor object
motor = Motor(MotorType, motor_id, feedback_id)

# Create MotorControl object
MotorControl = MotorControl(serial_device)
MotorControl.addMotor(motor)
MotorControl.switchControlMode(motor, Control_Type.VEL)
MotorControl.enable(motor)

time.sleep(0.2)
print("Motor enabled. Running test sequence...")

# =========================
# === MAIN LOOP ===========
# =========================
try:
    # Run motor forward at half speed for 3 seconds
    print("Motor forward at 3.0 rad/s...")
    MotorControl.control_Vel(motor, 3.0)
    time.sleep(3)
    
    # Stop motor for 1 second
    print("Motor stopped...")
    MotorControl.control_Vel(motor, 0.0)
    time.sleep(1)
    
    # Run motor backward at half speed for 3 seconds
    print("Motor backward at -3.0 rad/s...")
    MotorControl.control_Vel(motor, -3.0)
    time.sleep(3)
    
    # Stop motor
    print("Motor stopped...")
    MotorControl.control_Vel(motor, 0.0)
    time.sleep(1)
    
    print("Test sequence complete.")

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    # Stop and disable motor
    MotorControl.control_Vel(motor, 0)
    MotorControl.disable(motor)
    serial_device.close()
    print("Motor disabled, serial closed.")