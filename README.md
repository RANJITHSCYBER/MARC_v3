5-DOF Robotic Arm Controller
<p align="center"> <img src="https://img.shields.io/badge/version-3.0-blue" /> <img src="https://img.shields.io/badge/ESP32-Servo%20Controller-green" /> </p><p align="center"> Web-based controller for 5DOF robotic arm using ESP32. </p>
✨ Features
Feature	Description
Servo Control	Base, Shoulder, Elbow, Wrist, Gripper (5 servos)
Web Interface	Mobile & desktop compatible
Presets	Save/load 10 positions
Recording	Record/playback movements
Pot Control	4 axes with potentiometers
Button	Physical gripper control
Safety	Emergency stop
🔧 Hardware
Component	Quantity
ESP32	1
Servo motors	5
10k Potentiometers	4
Push button	1
LEDs	3
🔌 Pin Connections
Servo Connections
Servo	GPIO Pin
Base	25
Shoulder	26
Elbow	27
Wrist	14
Gripper	12
Potentiometer Connections
Pot	GPIO Pin
Base	32
Shoulder	33
Elbow	34
Wrist	35
LED Connections
LED	GPIO Pin
Green	5
Blue	18
Red	19
Button: GPIO 13

🚀 Quick Start
1. Install Libraries
text
ESP32Servo
ArduinoJson
2. Set WiFi in Code
cpp
const char* ssid = "your WiFi";
const char* password = "your password";
3. Upload to ESP32
4. Connect
URL: http://robotarm.local

AP Mode: RobotArm-Controller (password: admin123)

📱 Web Interface
Feature	Description
Sliders	Individual servo control
Quick Buttons	Preset positions
Presets	Save/load positions
Recording	Record/playback controls
Status	Real-time updates
📡 API Examples
Endpoint	Description
GET /api/servo/1?angle=90	Move base servo
GET /api/emergency	Emergency stop
POST /api/save_preset	Save current position
GET /api/record/start	Start recording
⚙️ Configurable Limits
cpp
#define GRIPPER_MAX   90   // Close
#define GRIPPER_MIN   0    // Open
#define WRIST_MAX     180  // Up
#define WRIST_MIN     0    // Down
🎯 Default Home Positions
Servo	Angle
Base	90°
Shoulder	90°
Elbow	90°
Wrist	90°
Gripper	45°
📊 Storage
Feature	Specification
Presets	10 positions (80 bytes each)
Recording	150 steps
Saving	Auto-saves to EEPROM
⚡ Power Requirements
Requirement	Specification
Voltage	5V
Current	5A+ recommended
Capacitor	Add 1000µF for stability
🔴 LED Indicators
LED	Meaning
Green	System ready
Blue	Recording active
Blue (blink)	Playback active
Red	Emergency stop
🛠️ Troubleshooting
Issue	Solution
Servos not moving	Check power, enable servos
No WiFi connection	Auto AP mode starts
Pots not working	Click "POT MODE" button
📝 License
MIT License

<p align="center"> Happy building! 🤖 </p>
