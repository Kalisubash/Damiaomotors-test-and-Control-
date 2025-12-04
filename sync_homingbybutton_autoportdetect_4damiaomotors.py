
import serial
import time
import math
import asyncio
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
# Async Motor Control Functions
# -----------------------------
async def control_motor_vel(MotorCtrl, motor, speed):
    """Async wrapper for controlling a single motor velocity"""
    await asyncio.sleep(0)  # Yield control
    MotorCtrl.control_Vel(motor, speed)

async def control_all_motors_forward_backward_async(MotorCtrl, motors, speed_fb):
    """Control all motors for forward/backward motion simultaneously"""
    tasks = [
        control_motor_vel(MotorCtrl, motors[0], speed_fb),
        control_motor_vel(MotorCtrl, motors[1], speed_fb),
        control_motor_vel(MotorCtrl, motors[2], -speed_fb),  # Inverted
        control_motor_vel(MotorCtrl, motors[3], -speed_fb),  # Inverted
    ]
    await asyncio.gather(*tasks)

async def control_all_motors_left_right_async(MotorCtrl, motors, speed_lr):
    """Control all motors for left/right motion simultaneously"""
    tasks = [control_motor_vel(MotorCtrl, motor, speed_lr) for motor in motors]
    await asyncio.gather(*tasks)

async def stop_all_motors_async(MotorCtrl, motors):
    """Stop all motors simultaneously"""
    tasks = [control_motor_vel(MotorCtrl, motor, 0) for motor in motors]
    await asyncio.gather(*tasks)

async def control_motors_to_target_async(MotorCtrl, motors, vel_commands):
    """Control all motors to target positions simultaneously"""
    tasks = [
        control_motor_vel(MotorCtrl, motors[i], vel_commands[i])
        for i in range(len(motors))
    ]
    await asyncio.gather(*tasks)

# -----------------------------
# Serial Port & Motor Setup
# -----------------------------
# Auto-detect the port
port = "/dev/DMmotorUSB"
try:
    ser = serial.Serial(port, 921600, timeout=0.5)
    print(f"Connected to {port}")
except Exception as e:
    print(f"Error connecting to {port}: {e}")
    exit()

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
# Joystick Configuration
# -----------------------------
MAX_SPEED = 6.0  # rad/s for directional movement
DEADZONE_RY_LOW = -300
DEADZONE_RY_HIGH = 0
DEADZONE_LX_LOW = -300
DEADZONE_LX_HIGH = 400

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

print("\n" + "="*60)
print("CONTROLS:")
print("  BTN_SOUTH (Button 0) : Move to closest target positions")
print("  Right Stick Y-axis   : Forward/Backward movement")
print("  Left Stick X-axis    : Left/Right movement")
print("  Press Ctrl+C to exit")
print("="*60 + "\n")

# -----------------------------
# Main async loop
# -----------------------------
async def main_loop():
    global next_loop_time, next_print_time
    
    moving_to_target = False
    target_degrees = [0, 0, 0, 0]  # Target for each motor
    button_pressed_last = False  # Track button state
    last_speed_fb = 0.0
    last_speed_lr = 0.0

    try:
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
            button_pressed = joystick.get_button(0)
            
            # Detect button press (rising edge)
            if button_pressed and not button_pressed_last:
                # Button just pressed - find closest target for each motor
                for i in range(4):
                    target_degrees[i] = closest_target(current_degrees[i], motor_positions[i])
                moving_to_target = True
                print(f"Button pressed! Moving to targets: M1={target_degrees[0]}Â°, M2={target_degrees[1]}Â°, M3={target_degrees[2]}Â°, M4={target_degrees[3]}Â°")
            
            button_pressed_last = button_pressed

            # Read joystick axes
            ry = joystick.get_axis(4)  # Right stick Y-axis
            lx = joystick.get_axis(0)  # Left stick X-axis
            
            # Convert to raw values similar to inputs library
            ry_raw = int(ry * 32767)
            lx_raw = int(lx * 32767)
            
            # Calculate Forward/Backward speed
            if DEADZONE_RY_LOW <= ry_raw <= DEADZONE_RY_HIGH:
                speed_fb = 0.0
            elif ry_raw < DEADZONE_RY_LOW:
                speed_fb = (ry_raw / -32800.0) * MAX_SPEED
            else:
                speed_fb = -(ry_raw / 32800.0) * MAX_SPEED
            
            # Calculate Left/Right speed
            if DEADZONE_LX_LOW <= lx_raw <= DEADZONE_LX_HIGH:
                speed_lr = 0.0
            elif lx_raw < DEADZONE_LX_LOW:
                speed_lr = (lx_raw / -32800.0) * MAX_SPEED
                speed_lr = -speed_lr  # Invert for left
            else:
                speed_lr = (lx_raw / 32800.0) * MAX_SPEED

            # Priority: Position targeting > Manual control
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
                
                # Send velocity commands to all motors simultaneously
                await control_motors_to_target_async(MotorCtrl, motors, vel_commands)
                
                if all_at_target:
                    moving_to_target = False
                    print("All motors reached target positions")
            
            elif abs(speed_fb) > 0.01 or abs(speed_lr) > 0.01:
                # Manual directional control - execute both simultaneously
                tasks = []
                
                if abs(speed_fb - last_speed_fb) > 0.01:
                    tasks.append(control_all_motors_forward_backward_async(MotorCtrl, motors, speed_fb))
                    last_speed_fb = speed_fb
                
                if abs(speed_lr - last_speed_lr) > 0.01:
                    tasks.append(control_all_motors_left_right_async(MotorCtrl, motors, speed_lr))
                    last_speed_lr = speed_lr
                
                if tasks:
                    await asyncio.gather(*tasks)
            
            else:
                # Motors idle - send zero velocity
                await stop_all_motors_async(MotorCtrl, motors)
                last_speed_fb = 0.0
                last_speed_lr = 0.0

            # Print status periodically
            if time.perf_counter() >= next_print_time:
                if moving_to_target:
                    status = "MOVING TO TARGET"
                elif abs(speed_fb) > 0.01 or abs(speed_lr) > 0.01:
                    status = f"MANUAL (FB:{speed_fb:.2f}, LR:{speed_lr:.2f})"
                else:
                    status = "IDLE"
                
                print(f"[{status}] M1:{current_degrees[0]:.1f}Â°â†’{target_degrees[0]}Â° | "
                      f"M2:{current_degrees[1]:.1f}Â°â†’{target_degrees[1]}Â° | "
                      f"M3:{current_degrees[2]:.1f}Â°â†’{target_degrees[2]}Â° | "
                      f"M4:{current_degrees[3]:.1f}Â°â†’{target_degrees[3]}Â°")
                next_print_time += print_interval

            # Maintain fixed loop timing
            sleep_time = next_loop_time - time.perf_counter()
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            next_loop_time += dt

    except KeyboardInterrupt:
        print("\nExiting...")

    finally:
        # Disable motors safely on exit
        print("\nDisabling motors...")
        await stop_all_motors_async(MotorCtrl, motors)
        await asyncio.sleep(0.2)
        
        for motor in motors:
            try:
                MotorCtrl.disable(motor)
                print(f"Motor {motor.SlaveID}/{motor.MasterID} DISABLED")
            except Exception as e:
                print(f"Error disabling Motor {motor.SlaveID}/{motor.MasterID}: {e}")
        ser.close()
        print("Motors disabled, serial closed")
        pygame.quit()

# -----------------------------
# Run the async main loop
# -----------------------------
if __name__ == "__main__":
    asyncio.run(main_loop())
