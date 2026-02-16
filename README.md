5-DOF Robotic Arm Controller
https://img.shields.io/badge/version-3.0-blue
https://img.shields.io/badge/ESP32-Servo%2520Controller-green

Web-based controller for 5DOF robotic arm using ESP32.

✨ Features
Control 5 servos (Base, Shoulder, Elbow, Wrist, Gripper)

Web interface (mobile/desktop)

Save/load 10 positions

Record/playback movements

Potentiometer control for 4 axes

Physical gripper button

Emergency stop

🔧 Hardware
Component	Qty
ESP32	1
Servo motors	5
10k Potentiometers	4
Push button	1
LEDs	3
🔌 Pin Connections
Servo	GPIO
Base	25
Shoulder	26
Elbow	27
Wrist	14
Gripper	12
Pot	GPIO
Base	32
Shoulder	33
Elbow	34
Wrist	35
LED	GPIO
Green	5
Blue	18
Red	19
Button: GPIO 13

🚀 Quick Start
Install libraries:

text
ESP32Servo, ArduinoJson
Set WiFi in code:

cpp
const char* ssid = "your WiFi";
const char* password = "your password";
Upload to ESP32

Connect to:

http://robotarm.local or

AP mode: RobotArm-Controller (pw: admin123)

📱 Web Interface
Sliders for each servo

Quick position buttons

Save/load presets

Record/playback controls

Real-time status

📡 API Examples
text
GET  /api/servo/1?angle=90     # Move base
GET  /api/emergency             # Stop all
POST /api/save_preset           # Save position
GET  /api/record/start          # Start recording
⚙️ Configurable Limits
cpp
#define GRIPPER_MAX  90   # Close
#define GRIPPER_MIN  0    # Open
#define WRIST_MAX    180  # Up
#define WRIST_MIN    0    # Down
🎯 Default Home
Servo	Angle
Base	90°
Shoulder	90°
Elbow	90°
Wrist	90°
Gripper	45°
📊 Storage
10 presets (80 bytes each)

150 recording steps

Auto-saves to EEPROM

⚡ Power
5V, 5A+ recommended

Add 1000µF capacitor

🔴 LED Indicators
LED	Meaning
Green	System ready
Blue	Recording
Blue blink	Playback
Red	Emergency
🛠️ Troubleshooting
Issue	Fix
Servos not moving	Check power, enable servos
No WiFi	Auto AP mode starts
Pots not working	Click "POT MODE"
📝 License
MIT

Happy building! 🤖
