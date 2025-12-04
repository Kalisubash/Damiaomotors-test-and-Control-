
import serial
import time
import math
from DM_CAN_Library import *
import pygame

# -----------------------------
# Helper functions
# -----------------------------
def normalize_angle_deg(angle_deg):
    """Normalize any angle to [0, 360) degrees."""
    return angle_deg % 360

def closest_target(current_deg, targets):
    """Return the closest target to current_deg."""
    diffs = [(abs(normalize_angle_deg(current_deg - t)), t) for t in targets]
    return min(diffs, key=lambda x: x[0])[1]

# -----------------------------
# Serial Port & Motor Setup
# -----------------------------
ser = serial.Serial("/dev/ttyACM0", 921600, timeout=0.5)

# Initialize all 4 motors
Motor1 = Motor(DM_Motor_Type.DM6006, SlaveID=0x01, MasterID=0x11)
Motor2 = Motor(DM_Motor_Type.DM6006, SlaveID=0x02, MasterID=0x12)
Motor3 = Motor(DM_Motor_Type.DM6006, SlaveID=0x03, MasterID=0x13)
Motor4 = Motor(DM_Motor_Type.DM6006, SlaveID=0x04, MasterID=0x14)

MotorCtrl = MotorControl(ser)

# Keep your own list of motors
motors = [Motor1, Motor2, Motor3, Motor4]

# Add motors to controller
for motor in motors:
    MotorCtrl.addMotor(motor)

# -----------------------------
# Robust enable sequence with prints
# -----------------------------
print("Disabling all motors first...")
for motor in motors:
    try:
        MotorCtrl.disable(motor)
        print(f"Motor {motor.SlaveID}/{motor.MasterID} DISABLED")
    except Exception as e:
        print(f"Error disabling Motor {motor.SlaveID}/{motor.MasterID}: {e}")
    time.sleep(0.1)

print("\nSetting velocity mode for all motors...")
for motor in motors:
    try:
        MotorCtrl.control_Mode(motor, DM_Control_Mode.DM_Velocity)
        print(f"Motor {motor.SlaveID}/{motor.MasterID} set to VELOCITY mode")
    except Exception as e:
        print(f"Error setting mode for Motor {motor.SlaveID}/{motor.MasterID}: {e}")
    time.sleep(0.1)

print("\nEnabling motors sequentially...")
for motor in motors:
    try:
        MotorCtrl.enable(motor)
        print(f"Motor {motor.SlaveID}/{motor.MasterID} ENABLED successfully")
    except Exception as e:
        print(f"Error enabling Motor {motor.SlaveID}/{motor.MasterID}: {e}")
    time.sleep(0.1)

# -----------------------------
# Predefined positions - SEPARATE FOR EACH MOTOR
# -----------------------------
positions_motor1 = [340, 100, 220]    # Motor 1 target positions in degrees
positions_motor2 = [320, 80, 200]     # Motor 2 target positions in degrees
positions_motor3 = [320, 80, 200]     # Motor 3 target positions in degrees
positions_motor4 = [35, 155, 275]     # Motor 4 target positions in degrees

# Store positions in a list for easy access
motor_positions = [positions_motor1, positions_motor2, positions_motor3, positions_motor4]

position_tolerance = 2.0  # degrees
max_velocity = 5.0        # rad/s max velocity
dt = 0.01
print_interval = 1.0

next_loop_time = time.perf_counter() + dt
next_print_time = time.perf_counter() + print_interval

# -----------------------------
# Initialize Joystick
# -----------------------------
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No joystick found!")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Joystick detected: {joystick.get_name()}")

# -----------------------------
# Main loop
# -----------------------------
try:
    moving_to_target = False
    target_degrees = [0, 0, 0, 0]  # Target for each motor
    button_pressed_last = False  # Track button state

    while True:
        MotorCtrl.recv()
        pygame.event.pump()

        # Read all motor positions in degrees
        current_degrees = [
            normalize_angle_deg(math.degrees(Motor1.getPosition())),
            normalize_angle_deg(math.degrees(Motor2.getPosition())),
            normalize_angle_deg(math.degrees(Motor3.getPosition())),
            normalize_angle_deg(math.degrees(Motor4.getPosition()))
        ]

        # Check if BTN_SOUTH (usually button 0) is pressed
        button_pressed = joystick.get_button(0)  # BTN_SOUTH is typically button 0
        
        # Detect button press (rising edge)
        if button_pressed and not button_pressed_last:
            # Button just pressed - find closest target for each motor
            for i in range(4):
                target_degrees[i] = closest_target(current_degrees[i], motor_positions[i])
            moving_to_target = True
            print(f"Button pressed! Moving to targets: M1={target_degrees[0]}Â°, M2={target_degrees[1]}Â°, M3={target_degrees[2]}Â°, M4={target_degrees[3]}Â°")
        
        button_pressed_last = button_pressed

        if moving_to_target:
            # Calculate velocity commands for all motors
            vel_commands = []
            all_at_target = True
            
            for i, motor in enumerate(motors):
                # Calculate error
                error_deg = normalize_angle_deg(target_degrees[i] - current_degrees[i])
                if error_deg > 180:
                    error_deg -= 360
                
                # Proportional control
                kp = 0.02
                vel_cmd = max(-max_velocity, min(max_velocity, kp * error_deg))
                
                # Stop if within tolerance
                if abs(error_deg) <= position_tolerance:
                    vel_cmd = 0
                else:
                    all_at_target = False
                
                vel_commands.append(vel_cmd)
            
            # Send velocity commands to all motors
            MotorCtrl.control_Vel(Motor1, vel_commands[0])
            MotorCtrl.control_Vel(Motor2, vel_commands[1])
            MotorCtrl.control_Vel(Motor3, vel_commands[2])
            MotorCtrl.control_Vel(Motor4, vel_commands[3])
            
            if all_at_target:
                moving_to_target = False
                print("All motors reached target positions")
        else:
            # Motors idle - send zero velocity
            MotorCtrl.control_Vel(Motor1, 0)
            MotorCtrl.control_Vel(Motor2, 0)
            MotorCtrl.control_Vel(Motor3, 0)
            MotorCtrl.control_Vel(Motor4, 0)

        # Print status periodically
        if time.perf_counter() >= next_print_time:
            status = "MOVING" if moving_to_target else "IDLE"
            print(f"[{status}] M1: {current_degrees[0]:.1f}Â°â†’{target_degrees[0]}Â° | "
                  f"M2: {current_degrees[1]:.1f}Â°â†’{target_degrees[1]}Â° | "
                  f"M3: {current_degrees[2]:.1f}Â°â†’{target_degrees[2]}Â° | "
                  f"M4: {current_degrees[3]:.1f}Â°â†’{target_degrees[3]}Â°")
            next_print_time += print_interval

        # Maintain fixed loop timing
        sleep_time = next_loop_time - time.perf_counter()
        if sleep_time > 0:
            time.sleep(sleep_time)
        next_loop_time += dt

except KeyboardInterrupt:
    print("Exiting...")

finally:
    # Disable motors safely on exit
    print("\nDisabling motors...")
    for motor in motors:
        try:
            MotorCtrl.disable(motor)
            print(f"Motor {motor.SlaveID}/{motor.MasterID} DISABLED")
        except Exception as e:
            print(f"Error disabling Motor {motor.SlaveID}/{motor.MasterID}: {e}")
    ser.close()
    print("Motors disabled, serial closed")
    pygame.quit()
