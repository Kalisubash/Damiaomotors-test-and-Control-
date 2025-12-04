
import time
import os
import glob
import asyncio
from DM_CAN_Library import *
import serial
from inputs import get_gamepad
from concurrent.futures import ThreadPoolExecutor

# =========================
# === PORT DETECTION ======
# =========================
def find_dm_motor_port():
    """
    Auto-detect DM Motor USB port
    Checks for /dev/DMmotorUSB first, then falls back to /dev/ttyACM*
    """
    # Check for symbolic link /dev/DMmotorUSB
    if os.path.exists("/dev/DMmotorUSB"):
        print(f"âœ“ Found DM Motor USB at: /dev/DMmotorUSB")
        return "/dev/DMmotorUSB"
    
    # Fallback: search for /dev/ttyACM* ports
    acm_ports = glob.glob("/dev/ttyACM*")
    
    if acm_ports:
        # Sort to get consistent ordering
        acm_ports.sort()
        print(f"âœ“ Found ACM ports: {acm_ports}")
        print(f"â†’ Using: {acm_ports[0]}")
        return acm_ports[0]
    
    # No ports found
    raise Exception("âœ— No DM Motor USB port found. Please check connection.")

# =========================
# === ASYNC MOTOR WRAPPER ==
# =========================
class AsyncMotorControl:
    """Async wrapper for DM Motor control"""
    
    def __init__(self, serial_device, motor_type):
        self.serial_device = serial_device
        self.motor_control = MotorControl(serial_device)
        self.motor_type = motor_type
        self.motors = []
        # Use single worker to prevent concurrent serial access
        self.executor = ThreadPoolExecutor(max_workers=1)
        self.lock = asyncio.Lock()
        self.command_queue = asyncio.Queue()
        
    async def add_motor_async(self, motor_id, feedback_id):
        """Add and initialize a motor asynchronously"""
        motor = Motor(self.motor_type, motor_id, feedback_id)
        self.motors.append(motor)
        
        async with self.lock:
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(self.executor, self.motor_control.addMotor, motor)
            await asyncio.sleep(0.05)
            await loop.run_in_executor(
                self.executor, 
                self.motor_control.switchControlMode, 
                motor, 
                Control_Type.VEL
            )
            await asyncio.sleep(0.05)
            await loop.run_in_executor(self.executor, self.motor_control.enable, motor)
            await asyncio.sleep(0.05)
        
        return motor
    
    async def control_vel_async(self, motor, velocity):
        """Send velocity command asynchronously with serial lock"""
        async with self.lock:
            loop = asyncio.get_event_loop()
            try:
                await loop.run_in_executor(
                    self.executor,
                    self.motor_control.control_Vel,
                    motor,
                    velocity
                )
            except Exception as e:
                print(f"Warning: Failed to send velocity to motor: {e}")
                # Try to recover
                await asyncio.sleep(0.1)
    
    async def control_vel_batch_async(self, motor_velocity_pairs):
        """Send multiple velocity commands in one serial transaction"""
        async with self.lock:
            loop = asyncio.get_event_loop()
            try:
                for motor, velocity in motor_velocity_pairs:
                    await loop.run_in_executor(
                        self.executor,
                        self.motor_control.control_Vel,
                        motor,
                        velocity
                    )
                    await asyncio.sleep(0.002)  # Small delay between commands
            except Exception as e:
                print(f"Warning: Batch command failed: {e}")
    
    async def disable_motor_async(self, motor):
        """Disable motor asynchronously"""
        async with self.lock:
            loop = asyncio.get_event_loop()
            try:
                await loop.run_in_executor(self.executor, self.motor_control.control_Vel, motor, 0)
                await asyncio.sleep(0.05)
                await loop.run_in_executor(self.executor, self.motor_control.disable, motor)
                await asyncio.sleep(0.05)
            except Exception as e:
                print(f"Warning: Failed to disable motor: {e}")
    
    def close(self):
        """Shutdown executor"""
        self.executor.shutdown(wait=True)

# =========================
# === MOTOR CONFIG ========
# =========================
try:
    serial_port = find_dm_motor_port()
except Exception as e:
    print(e)
    print("\nTroubleshooting:")
    print("1. Check if USB cable is connected")
    print("2. Check USB permissions: sudo chmod 666 /dev/ttyACM*")
    print("3. Create udev rule for /dev/DMmotorUSB")
    exit(1)

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
# === ASYNC MOTOR CONTROL FUNCTIONS ===
# =========================
async def control_all_motors_forward_backward(motor_control, motors, speed_fb):
    """Control all motors for forward/backward motion using batch command"""
    motor_velocity_pairs = []
    
    for idx, motor in enumerate(motors):
        if idx >= 2:  # Motors 3 and 4 are inverted
            motor_velocity_pairs.append((motor, -speed_fb))
        else:
            motor_velocity_pairs.append((motor, speed_fb))
    
    # Send all commands in one batch (serialized but fast)
    await motor_control.control_vel_batch_async(motor_velocity_pairs)

async def control_all_motors_left_right(motor_control, motors, speed_lr):
    """Control all motors for left/right motion using batch command"""
    motor_velocity_pairs = [(motor, speed_lr) for motor in motors]
    
    # Send all commands in one batch
    await motor_control.control_vel_batch_async(motor_velocity_pairs)

async def stop_all_motors(motor_control, motors):
    """Stop all motors using batch command"""
    motor_velocity_pairs = [(motor, 0.0) for motor in motors]
    await motor_control.control_vel_batch_async(motor_velocity_pairs)

# =========================
# === INITIALIZE MOTORS ===
# =========================
async def initialize_motors():
    """Initialize all motors asynchronously"""
    print(f"\nConnecting to {serial_port} at {baud_rate} baud...")
    
    try:
        # Open serial with exclusive access
        serial_device = serial.Serial(
            serial_port, 
            baud_rate, 
            timeout=0.5,
            exclusive=True  # Prevent multiple access
        )
        print("âœ“ Serial connection established")
        
        # Clear any pending data
        serial_device.reset_input_buffer()
        serial_device.reset_output_buffer()
        
    except serial.SerialException as e:
        if "Permission denied" in str(e):
            print(f"âœ— Permission denied. Try: sudo chmod 666 {serial_port}")
        elif "exclusive" in str(e).lower():
            print(f"âœ— Port already in use. Close other programs accessing {serial_port}")
        else:
            print(f"âœ— Failed to open serial port: {e}")
        exit(1)
    except Exception as e:
        print(f"âœ— Failed to open serial port: {e}")
        exit(1)
    
    # Create async motor control
    motor_control = AsyncMotorControl(serial_device, MotorType)
    
    print("\nInitializing motors sequentially (to prevent serial conflicts)...")
    motors = []
    
    # Initialize motors one by one to avoid serial port conflicts
    for m_id, f_id in zip(motor_ids, feedback_ids):
        print(f"  â†’ Motor ID: {hex(m_id)}, Feedback ID: {hex(f_id)}")
        motor = await motor_control.add_motor_async(m_id, f_id)
        motors.append(motor)
        await asyncio.sleep(0.1)
    
    await asyncio.sleep(0.2)
    print("âœ“ All 4 motors enabled")
    
    return motor_control, motors, serial_device

# =========================
# === MAIN CONTROL LOOP ===
# =========================
async def main_control_loop(motor_control, motors):
    """Main asynchronous control loop"""
    print("\n" + "="*60)
    print("JOYSTICK CONTROLS:")
    print("  Right Stick Y-axis  : Forward/Backward")
    print("  Left Stick X-axis   : Left/Right")
    print("  Press Ctrl+C to exit")
    print("="*60 + "\n")
    
    loop = asyncio.get_event_loop()
    last_speed_fb = 0.0
    last_speed_lr = 0.0
    
    try:
        while True:
            # Get gamepad events (blocking, so run in executor)
            events = await loop.run_in_executor(None, get_gamepad)
            
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
                    
                    # Only send command if speed changed
                    if abs(speed_fb - last_speed_fb) > 0.01:
                        await control_all_motors_forward_backward(motor_control, motors, speed_fb)
                        last_speed_fb = speed_fb
                        # print(f"RY: {ry:6d}  â†’ FB Speed: {speed_fb:.2f} rad/s")
                
                # --- Left/Right with Left Joystick X-axis ---
                elif event.code == "ABS_X":
                    lx = event.state
                    
                    # Deadzone for small X values
                    if DEADZONE_LX_LOW <= lx <= DEADZONE_LX_HIGH:
                        speed_lr = 0.0
                    elif lx < DEADZONE_LX_LOW:
                        # Move left
                        speed_lr = (lx / -32800.0) * MAX_SPEED
                        speed_lr = -speed_lr  # Invert for left
                    else:  # lx > DEADZONE_LX_HIGH
                        # Move right
                        speed_lr = (lx / 32800.0) * MAX_SPEED
                    
                    # Only send command if speed changed
                    if abs(speed_lr - last_speed_lr) > 0.01:
                        await control_all_motors_left_right(motor_control, motors, speed_lr)
                        last_speed_lr = speed_lr
                        print(f"LX: {lx:6d}  â†’ LR Speed: {speed_lr:.2f} rad/s")
            
            # Small delay to prevent CPU overload
            await asyncio.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\n\nâš  Stopping...")
        raise

# =========================
# === SHUTDOWN ===
# =========================
async def shutdown_motors(motor_control, motors, serial_device):
    """Shutdown all motors gracefully"""
    print("\n[Shutdown Sequence]")
    
    try:
        # Stop all motors
        print("  â†’ Stopping all motors...")
        await stop_all_motors(motor_control, motors)
        await asyncio.sleep(0.2)
        
        # Disable motors one by one to avoid serial conflicts
        print("  â†’ Disabling motors...")
        for idx, motor in enumerate(motors):
            try:
                await motor_control.disable_motor_async(motor)
                print(f"  âœ“ Motor {idx+1} (ID: {hex(motor_ids[idx])}) disabled")
            except Exception as e:
                print(f"  âœ— Motor {idx+1} disable error: {e}")
        
        motor_control.close()
        
    except Exception as e:
        print(f"  âš  Shutdown error: {e}")
    
    finally:
        try:
            serial_device.close()
            print("âœ“ Serial connection closed")
        except:
            pass
        print("\nProgram ended.")

# =========================
# === MAIN ASYNC FUNCTION ===
# =========================
async def main_async():
    """Main async entry point"""
    motor_control = None
    motors = None
    serial_device = None
    
    try:
        # Initialize motors
        motor_control, motors, serial_device = await initialize_motors()
        
        # Run main control loop
        await main_control_loop(motor_control, motors)
        
    except KeyboardInterrupt:
        pass
    
    finally:
        if motor_control and motors and serial_device:
            await shutdown_motors(motor_control, motors, serial_device)

# =========================
# === ENTRY POINT ===
# =========================
if __name__ == "__main__":
    asyncio.run(main_async())
