# Damiaomotors-test-and-Control-
In this repository all the code are posted that were used dusing testing ot damaio motors
This project demonstrates how to control four Damiao (DM6006) brushless servo motors using a Logitech Wireless Gamepad F710.
The system reads joystick inputs in real-time using the inputs Python library and maps those values to motor velocity commands through the DM_CAN_Library.

The script includes:
Real-time gamepad reading
Deadzone filtering for smooth control
Forward/reverse movement using right joystick (ABS_RY)
Left/right turning using left joystick (ABS_X)
Inverted motor logic for mecanum or differential robots
Safe motor enable/disable
USB-CAN communication
Support for Raspberry Pi or PC

This repository is useful for building:
Mecanum drive robots
Differential drive robots
Robotic platforms
Actuator systems

Research and prototyping with CAN-controlled motors

📦 Required Python Libraries
python -m pip install --upgrade pip
sudo apt update
python3 -m pip install --upgrade pip
pip install pyserial
pip install inputs
pip install pyserial pygame inputs



Ensure DM_CAN_Library.py is included in the folder.

🔌 Hardware Requirements
4× Damiao DM6006 motors
USB-CAN adapter (SlCAN, CANable, Waveshare, etc.)
120 Ω termination resistor (required!)
Logitech F710 Wireless Gamepad
Raspberry Pi 4 or PC
Stable motor power supply

🔌 CAN Bus Wiring & Termination
CANH → CANH  
CANL → CANL  
GND  → GND  
Motor Power → External Supply  
USB-CAN → Raspberry Pi / PC

A 120Ω termination resistor is required at each end of the CAN line.
Without it, motors may not respond or CAN errors may appear.

(https://github.com/Kalisubash/Damiaomotors-test-and-Control-/blob/main/pi_1damiaomotortest.py)
--this code run one damiaomotor of can id 01 forward and backward with damiao debugger . the images of debugger posted in this repository https://github.com/Kalisubash/Damiaomotors-test-and-Control-/blob/main/Damiao%20can%20debugger.jpeg  . for one motor there is no necessary of terminal motor 

https://github.com/Kalisubash/Damiaomotors-test-and-Control-/blob/main/pi_2damiaomotorstest.py
-- this code run two damiaomotor of can id 01 and 02 forward and backward with damiao debugger . the images of debugger posted in this repository https://github.com/Kalisubash/Damiaomotors-test-and-Control-/blob/main/Damiao%20can%20debugger.jpeg  . for two or more motors you have to connect the terminal resistor and about connection shown in https://github.com/Kalisubash/Damiaomotors-test-and-Control-/blob/main/how%20to%20connect%20terminal%20resistor.PNG

https://github.com/Kalisubash/Damiaomotors-test-and-Control-/blob/main/pi_4damiaomotors_with_logitechgamepad.py
--this code run 4 damiaomotor of can id 01 ,02,03 and 04 forward and backward with damiao debugger with logitech gamepad F710 . the images of debugger posted in this repository https://github.com/Kalisubash/Damiaomotors-test-and-Control-/blob/main/Damiao%20can%20debugger.jpeg  . for two or more motors you have to connect the terminal resistor and about connection shown in https://github.com/Kalisubash/Damiaomotors-test-and-Control-/blob/main/how%20to%20connect%20terminal%20resistor.PNG

https://github.com/Kalisubash/Damiaomotors-test-and-Control-/blob/main/autoportdetect_4damiaomotors.py
-- this code run 4 damiaomotor of can id 01 ,02,03 and 04 forward and backward with damiao debugger with logitech gamepad F710 . the images of debugger posted in this repository https://github.com/Kalisubash/Damiaomotors-test-and-Control-/blob/main/Damiao%20can%20debugger.jpeg  . for two or more motors you have to connect the terminal resistor and about connection shown in https://github.com/Kalisubash/Damiaomotors-test-and-Control-/blob/main/how%20to%20connect%20terminal%20resistor.PNG
also in this code the usb port detect automatically even if we connect other more usb and communicate

https://github.com/Kalisubash/Damiaomotors-test-and-Control-/blob/main/sync_autoportdetect_4damiaomotors.py
-- this code run 4 damiaomotor of can id 01 ,02,03 and 04 forward and backward with damiao debugger with logitech gamepad F710 . the images of debugger posted in this repository https://github.com/Kalisubash/Damiaomotors-test-and-Control-/blob/main/Damiao%20can%20debugger.jpeg  . for two or more motors you have to connect the terminal resistor and about connection shown in https://github.com/Kalisubash/Damiaomotors-test-and-Control-/blob/main/how%20to%20connect%20terminal%20resistor.PNG
also in this code the usb port detect automatically even if we connect other more usb and communicate and run all motors simulataneoouly

https://github.com/Kalisubash/Damiaomotors-test-and-Control-/blob/main/Homing_by_a_button.py
-- --this code run 4 damiaomotor of can id 01 ,02,03 and 04 forward and backward with damiao debugger with logitech gamepad F710 . the images of debugger posted in this repository https://github.com/Kalisubash/Damiaomotors-test-and-Control-/blob/main/Damiao%20can%20debugger.jpeg  . for two or more motors you have to connect the terminal resistor and about connection shown in https://github.com/Kalisubash/Damiaomotors-test-and-Control-/blob/main/how%20to%20connect%20terminal%20resistor.PNG 
along this the motor can be get in homing position in specific angle by clicking a single button

https://github.com/Kalisubash/Damiaomotors-test-and-Control-/blob/main/sync_homingbybutton_autoportdetect_4damiaomotors.py
-- this code run 4 damiaomotor of can id 01 ,02,03 and 04 forward and backward with damiao debugger with logitech gamepad F710 . the images of debugger posted in this repository https://github.com/Kalisubash/Damiaomotors-test-and-Control-/blob/main/Damiao%20can%20debugger.jpeg  . for two or more motors you have to connect the terminal resistor and about connection shown in https://github.com/Kalisubash/Damiaomotors-test-and-Control-/blob/main/how%20to%20connect%20terminal%20resistor.PNG
also in this code the usb port detect automatically even if we connect other more usb and communicate and run all motors simulataneoouly.
along this the motor can be get in homing position in specific angle by clicking a single button




