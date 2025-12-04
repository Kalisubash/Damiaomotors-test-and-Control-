
import can
import struct
import time
import math
import asyncio
import os
import glob
from inputs import get_gamepad
from concurrent.futures import ThreadPoolExecutor

# =========================
# === PORT DETECTION ======
# =========================
def find_cybergear_port():
    """
    Auto-detect CyberGear USB port
    Checks for /dev/CyberGearUSB first, then falls back to /dev/ttyACM*
    """
    # Check for symbolic link /dev/CyberGearUSB
    if os.path.exists("/dev/CyberGearUSB"):
        print(f"âœ“ Found CyberGear USB at: /dev/CyberGearUSB")
        return "/dev/CyberGearUSB"
    
    # Fallback: search for /dev/ttyACM* ports
    acm_ports = glob.glob("/dev/ttyACM*")
    
    if acm_ports:
        # Sort to get consistent ordering
        acm_ports.sort()
        print(f"âœ“ Found ACM ports: {acm_ports}")
        
        # If multiple ports, try to identify CyberGear
        if len(acm_ports) > 1:
            print(f"â†’ Multiple ports found, using: {acm_ports[-1]} (last port)")
            return acm_ports[-1]
        else:
            print(f"â†’ Using: {acm_ports[0]}")
            return acm_ports[0]
    
    # No ports found
    raise Exception("âœ— No CyberGear USB port found. Please check connection.")

class CyberGearMotor:
    def __init__(self, interface='slcan', channel=None, bitrate=1000000):
        """
        Initialize CyberGear motor with USB-CAN adapter on Raspberry Pi
        """
        # Auto-detect channel if not provided
        if channel is None:
            try:
                channel = find_cybergear_port()
            except Exception as e:
                print(e)
                print("\nTroubleshooting:")
                print("1. Check if USB cable is connected")
                print("2. Check USB permissions: sudo chmod 666 /dev/ttyACM*")
                print("3. Create udev rule for /dev/CyberGearUSB")
                raise
        
        try:
            self.bus = can.interface.Bus(
                channel=channel,
                interface=interface,
                bitrate=bitrate,
                ttyBaudrate=115200
            )
            print(f"âœ“ Connected to {interface}:{channel} at {bitrate} bps")
        except Exception as e:
            raise Exception(f"Failed to connect to CAN bus: {e}")
        
        self.master_id = 0x00FD
        self.last_positions = {}  # Track last sent position for each motor
        self.executor = ThreadPoolExecutor(max_workers=4)
        self.lock = asyncio.Lock()
    
    async def send_can_frame_async(self, cmd_id, motor_id, data, retry=3):
        """Send CAN frame asynchronously with retry logic"""
        can_id = (cmd_id << 24) | (self.master_id << 8) | motor_id
        
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=True
        )
        
        loop = asyncio.get_event_loop()
        
        for attempt in range(retry):
            try:
                # Run blocking CAN send in executor
                await loop.run_in_executor(self.executor, self.bus.send, msg)
                await asyncio.sleep(0.02)  # Increased delay for reliability
                return True
            except Exception as e:
                if attempt == retry - 1:
                    print(f"Error sending CAN frame to motor {motor_id} after {retry} attempts: {e}")
                    return False
                await asyncio.sleep(0.08)  # Longer delay before retry
        return False
    
    async def clear_can_buffer_async(self):
        """Clear any pending CAN messages asynchronously"""
        loop = asyncio.get_event_loop()
        try:
            while True:
                msg = await loop.run_in_executor(
                    self.executor, 
                    lambda: self.bus.recv(timeout=0.01)
                )
                if msg is None:
                    break
        except:
            pass
    
    async def enable_motor_async(self, motor_id):
        """Enable motor asynchronously - Command ID: 0x03"""
        data = [0x00] * 8
        await self.send_can_frame_async(0x03, motor_id, data)
        await asyncio.sleep(0.3)
        print(f"  âœ“ Motor {motor_id} enabled")
    
    async def disable_motor_async(self, motor_id):
        """Disable motor asynchronously - Command ID: 0x04"""
        data = [0x00] * 8
        await self.send_can_frame_async(0x04, motor_id, data)
        await asyncio.sleep(0.3)
        print(f"  âœ“ Motor {motor_id} disabled")
    
    async def stop_motor_async(self, motor_id):
        """Stop motor immediately asynchronously"""
        data = [0x00] * 8
        await self.send_can_frame_async(0x00, motor_id, data)
        await asyncio.sleep(0.1)
        print(f"  âœ“ Motor {motor_id} stopped")
    
    async def reset_position_async(self, motor_id):
        """Reset position to zero asynchronously - Command ID: 0x06"""
        data = [0x01] + [0x00] * 7
        await self.send_can_frame_async(0x06, motor_id, data)
        async with self.lock:
            self.last_positions[motor_id] = 0.0
        await asyncio.sleep(0.3)
        print(f"  âœ“ Motor {motor_id} position reset to zero")
    
    async def set_position_control_mode_async(self, motor_id):
        """Set motor to position control mode asynchronously"""
        data = [0x05, 0x70, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00]
        await self.send_can_frame_async(0x12, motor_id, data)
        await asyncio.sleep(0.2)
        print(f"  âœ“ Motor {motor_id} set to position control mode")
    
    async def set_limit_speed_async(self, motor_id, speed_rad_s):
        """Set speed limit asynchronously"""
        speed_bytes = struct.pack('<f', speed_rad_s)
        data = [0x17, 0x70, 0x00, 0x00] + list(speed_bytes)
        await self.send_can_frame_async(0x12, motor_id, data)
        await asyncio.sleep(0.05)
    
    async def set_limit_torque_async(self, motor_id, torque_nm):
        """Set torque limit asynchronously"""
        torque_bytes = struct.pack('<f', torque_nm)
        data = [0x18, 0x70, 0x00, 0x00] + list(torque_bytes)
        await self.send_can_frame_async(0x12, motor_id, data)
        await asyncio.sleep(0.05)
    
    async def set_target_position_async(self, motor_id, radians):
        """Set target position asynchronously - only send if different from last"""
        # Check if position has actually changed
        should_send = True
        async with self.lock:
            if motor_id in self.last_positions:
                if abs(self.last_positions[motor_id] - radians) < 0.001:
                    should_send = False
        
        if not should_send:
            return True  # Position unchanged, consider it success
        
        pos_bytes = struct.pack('<f', radians)
        data = [0x16, 0x70, 0x00, 0x00] + list(pos_bytes)
        success = await self.send_can_frame_async(0x12, motor_id, data)
        
        if success:
            async with self.lock:
                self.last_positions[motor_id] = radians
        else:
            print(f"  âš  Failed to set position for motor {motor_id}")
        
        await asyncio.sleep(0.03)
        return success
    
    async def move_to_position_async(self, motor_id, radians, speed_rad_s=5.0, torque_nm=12.0, force=False, max_retries=3):
        """
        Move motor to target position asynchronously with retry logic
        
        Args:
            force: If True, resend limits even if position is the same
            max_retries: Number of times to retry if command fails
        """
        # Check if we need to move
        if not force:
            async with self.lock:
                if motor_id in self.last_positions:
                    if abs(self.last_positions[motor_id] - radians) < 0.001:
                        return True  # Already at position
        
        # Retry loop for robustness
        for attempt in range(max_retries):
            try:
                # Set limits and position with small delays
                await self.set_limit_torque_async(motor_id, torque_nm)
                await asyncio.sleep(0.02)  # Small delay between commands
                await self.set_limit_speed_async(motor_id, speed_rad_s)
                await asyncio.sleep(0.02)
                success = await self.set_target_position_async(motor_id, radians)
                
                if success:
                    return True
                else:
                    if attempt < max_retries - 1:
                        print(f"  âš  Motor {motor_id} command failed, retrying ({attempt + 1}/{max_retries})...")
                        await asyncio.sleep(0.1)
            except Exception as e:
                if attempt < max_retries - 1:
                    print(f"  âš  Motor {motor_id} error: {e}, retrying ({attempt + 1}/{max_retries})...")
                    await asyncio.sleep(0.1)
                else:
                    print(f"  âœ— Motor {motor_id} failed after {max_retries} attempts")
                    return False
        
        return False
    
    async def move_to_degrees_async(self, motor_id, degrees, speed_rad_s=5.0, torque_nm=12.0, force=False, max_retries=3):
        """Move motor to target position in degrees asynchronously"""
        radians = math.radians(degrees)
        return await self.move_to_position_async(motor_id, radians, speed_rad_s, torque_nm, force, max_retries)
    
    async def refresh_motor_async(self, motor_id, speed_rad_s=5.0, torque_nm=12.0):
        """Refresh motor state by resending current position with limits asynchronously"""
        async with self.lock:
            if motor_id in self.last_positions:
                current_rad = self.last_positions[motor_id]
            else:
                return
        
        await self.set_limit_torque_async(motor_id, torque_nm)
        await self.set_limit_speed_async(motor_id, speed_rad_s)
        # Force resend of position
        pos_bytes = struct.pack('<f', current_rad)
        data = [0x16, 0x70, 0x00, 0x00] + list(pos_bytes)
        await self.send_can_frame_async(0x12, motor_id, data)
    
    async def emergency_stop_async(self, motor_id):
        """Emergency stop asynchronously"""
        data = [0x00] * 8
        await self.send_can_frame_async(0x00, motor_id, data)
        print(f"  âš  Motor {motor_id} EMERGENCY STOP")
    
    def close(self):
        """Close CAN connection"""
        if hasattr(self, 'bus'):
            self.bus.shutdown()
            print("âœ“ CAN connection closed")
        if hasattr(self, 'executor'):
            self.executor.shutdown(wait=True)


async def initialize_motors(motor, motor_ids, speed, max_torque):
    """Initialize all motors simultaneously"""
    print("\n[Initialization - Enabling Motors]")
    await asyncio.gather(*[motor.enable_motor_async(mid) for mid in motor_ids])
    await asyncio.sleep(0.5)
    
    print("\n[Resetting Positions]")
    await asyncio.gather(*[motor.reset_position_async(mid) for mid in motor_ids])
    await asyncio.sleep(0.5)
    
    print("\n[Setting Position Control Mode]")
    await asyncio.gather(*[motor.set_position_control_mode_async(mid) for mid in motor_ids])
    await asyncio.sleep(0.5)
    
    print("\n[Setting Initial Limits]")
    tasks = []
    for motor_id in motor_ids:
        tasks.append(motor.set_limit_speed_async(motor_id, speed))
        tasks.append(motor.set_limit_torque_async(motor_id, max_torque))
    await asyncio.gather(*tasks)


async def move_motors_simultaneously(motor, forward_motors, reverse_motors, position, speed, torque):
    """Move all motors to their target positions simultaneously with verification"""
    tasks = []
    
    for motor_id in forward_motors:
        tasks.append(motor.move_to_degrees_async(motor_id, position, speed, torque, max_retries=3))
    
    for motor_id in reverse_motors:
        tasks.append(motor.move_to_degrees_async(motor_id, -position, speed, torque, max_retries=3))
    
    results = await asyncio.gather(*tasks, return_exceptions=True)
    
    # Check if any motor failed
    failed_motors = []
    all_motor_ids = forward_motors + reverse_motors
    for i, result in enumerate(results):
        if isinstance(result, Exception) or result == False:
            failed_motors.append(all_motor_ids[i])
    
    if failed_motors:
        print(f"  âš  Warning: Motors {failed_motors} may not have responded correctly")
        print(f"  â†’ Sending redundant command to all motors...")
        
        # Resend to ALL motors as a safety measure
        retry_tasks = []
        for motor_id in forward_motors:
            retry_tasks.append(motor.move_to_degrees_async(motor_id, position, speed, torque, force=True, max_retries=2))
        for motor_id in reverse_motors:
            retry_tasks.append(motor.move_to_degrees_async(motor_id, -position, speed, torque, force=True, max_retries=2))
        
        await asyncio.gather(*retry_tasks, return_exceptions=True)
        print(f"  âœ“ Redundant command sent")
    
    return len(failed_motors) == 0


async def refresh_all_motors(motor, motor_ids, speed, torque):
    """Refresh all motors simultaneously"""
    await asyncio.gather(*[motor.refresh_motor_async(mid, speed, torque) for mid in motor_ids])


async def shutdown_motors(motor, motor_ids):
    """Shutdown all motors simultaneously"""
    print("\n[Stopping Motors]")
    await asyncio.gather(*[motor.stop_motor_async(mid) for mid in motor_ids])
    await asyncio.sleep(0.3)
    
    print("\n[Disabling Motors]")
    await asyncio.gather(*[motor.disable_motor_async(mid) for mid in motor_ids])
    await asyncio.sleep(0.3)


async def gamepad_handler(motor, motor_ids, forward_motors, reverse_motors, speed, max_torque, 
                          min_pos, max_pos, increment, refresh_interval):
    """Handle gamepad input asynchronously"""
    current_position = 0
    last_hat0y = 0
    last_hat0x = 0
    last_refresh_time = time.time()
    
    print(f"\nâœ“ All motors initialized at {current_position}Â°")
    print("\n" + "="*60)
    print("Waiting for gamepad input... (Press Ctrl+C to exit)")
    print("="*60 + "\n")
    
    loop = asyncio.get_event_loop()
    
    while True:
        try:
            # Periodic motor state refresh
            current_time = time.time()
            if current_time - last_refresh_time > refresh_interval:
                print(f"\n[Auto-refresh] Refreshing motor states...")
                await refresh_all_motors(motor, motor_ids, speed, max_torque)
                last_refresh_time = current_time
                await motor.clear_can_buffer_async()
            
            # Get gamepad events in executor to avoid blocking
            events = await loop.run_in_executor(None, get_gamepad)
            
            for event in events:
                if event.code == 'ABS_HAT0Y':
                    # UP pressed
                    if event.state == -1 and last_hat0y != -1:
                        new_position = current_position + increment
                        
                        if new_position <= max_pos:
                            current_position = new_position
                            print(f"\n[D-PAD UP] Moving to {current_position}Â°")
                            
                            await move_motors_simultaneously(
                                motor, forward_motors, reverse_motors, 
                                current_position, speed, max_torque
                            )
                            await asyncio.sleep(0.15)  # Small delay after movement
                            
                            print(f"  â†’ Motors moving...")
                        else:
                            print(f"\nâš  Already at maximum ({max_pos}Â°)")
                    
                    # DOWN pressed
                    elif event.state == 1 and last_hat0y != 1:
                        new_position = current_position - increment
                        
                        if new_position >= min_pos:
                            current_position = new_position
                            print(f"\n[D-PAD DOWN] Moving to {current_position}Â°")
                            
                            await move_motors_simultaneously(
                                motor, forward_motors, reverse_motors, 
                                current_position, speed, max_torque
                            )
                            await asyncio.sleep(0.15)  # Small delay after movement
                            
                            print(f"  â†’ Motors moving...")
                        else:
                            print(f"\nâš  Already at minimum ({min_pos}Â°)")
                    
                    last_hat0y = event.state
                
                elif event.code == 'ABS_HAT0X':
                    # LEFT pressed
                    if event.state == -1 and last_hat0x != -1:
                        if current_position != max_pos:
                            current_position = max_pos
                            print(f"\n[D-PAD LEFT] Jumping to {current_position}Â°")
                            
                            await move_motors_simultaneously(
                                motor, forward_motors, reverse_motors, 
                                current_position, speed, max_torque
                            )
                            await asyncio.sleep(0.15)  # Small delay after movement
                            
                            print(f"  â†’ Motors moving...")
                        else:
                            print(f"\nâš  Already at {max_pos}Â°")
                    
                    # RIGHT pressed
                    elif event.state == 1 and last_hat0x != 1:
                        if current_position != min_pos:
                            current_position = min_pos
                            print(f"\n[D-PAD RIGHT] Returning to neutral (0Â°)")
                            
                            # All motors go to 0
                            await asyncio.gather(*[
                                motor.move_to_degrees_async(mid, 0, speed, max_torque, max_retries=3) 
                                for mid in motor_ids
                            ])
                            await asyncio.sleep(0.15)  # Small delay after movement
                            
                            print(f"  â†’ Motors moving to neutral...")
                        else:
                            print(f"\nâš  Already at {min_pos}Â°")
                    
                    last_hat0x = event.state
            
        except Exception as e:
            if "device" in str(e).lower():
                print("\nâš  Gamepad disconnected.")
                break
            else:
                raise


async def main_async():
    # Configuration
    INTERFACE = 'slcan'
    CHANNEL = None  # Auto-detect
    
    MOTOR_IDS = [1, 2, 3, 4]
    FORWARD_MOTORS = [1, 2]
    REVERSE_MOTORS = [3, 4]
    
    SPEED = 2.0
    MAX_TORQUE = 12.0
    
    MIN_POSITION = 0
    MAX_POSITION = 45
    INCREMENT = 15
    
    REFRESH_INTERVAL = 2.0
    
    # Reliability settings
    COMMAND_RETRY_COUNT = 3  # Number of retries for failed commands
    INTER_COMMAND_DELAY = 0.15  # Delay between command sequences (seconds)
    
    print("\n" + "="*60)
    print("CyberGear 4-Motor Controller - Async Enhanced Version")
    print("="*60)
    print(f"Interface: {INTERFACE}")
    print(f"Channel: Auto-detect")
    print(f"Forward Motors: {FORWARD_MOTORS}")
    print(f"Reverse Motors: {REVERSE_MOTORS}")
    print(f"Retry Count: {COMMAND_RETRY_COUNT}")
    print("\n" + "="*60)
    print("GAMEPAD CONTROLS:")
    print("  D-PAD UP    : +15Â°")
    print("  D-PAD DOWN  : -15Â°")
    print("  D-PAD LEFT  : Jump to +45Â°")
    print("  D-PAD RIGHT : Jump to 0Â° (neutral)")
    print("="*60 + "\n")
    
    motor = None
    
    try:
        motor = CyberGearMotor(interface=INTERFACE, channel=CHANNEL)
        
        # Initialize all motors simultaneously
        await initialize_motors(motor, MOTOR_IDS, SPEED, MAX_TORQUE)
        
        # Run gamepad handler
        await gamepad_handler(
            motor, MOTOR_IDS, FORWARD_MOTORS, REVERSE_MOTORS,
            SPEED, MAX_TORQUE, MIN_POSITION, MAX_POSITION,
            INCREMENT, REFRESH_INTERVAL
        )
        
    except KeyboardInterrupt:
        print("\n\nâš  Shutdown initiated by user")
        print("\n[Shutdown Sequence]")
        if motor:
            try:
                await shutdown_motors(motor, MOTOR_IDS)
                motor.close()
                print("\nâœ“ Shutdown completed successfully")
            except Exception as shutdown_error:
                print(f"âœ— Error during shutdown: {shutdown_error}")
        
    except Exception as e:
        print(f"\nâœ— Error: {e}")
        import traceback
        traceback.print_exc()
        if motor:
            try:
                print("\n[Emergency Stop]")
                await asyncio.gather(*[
                    motor.emergency_stop_async(mid) for mid in MOTOR_IDS
                ])
                await asyncio.gather(*[
                    motor.disable_motor_async(mid) for mid in MOTOR_IDS
                ])
            except:
                pass
    
    finally:
        print("\n" + "="*60)
        print("Program ended")
        print("="*60)


if __name__ == "__main__":
    asyncio.run(main_async())
