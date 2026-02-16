5-DOF Robotic Arm Controller for ESP32
https://img.shields.io/badge/version-3.0-blue
https://img.shields.io/badge/platform-ESP32-green
https://img.shields.io/badge/servos-5%2520DOF-orange

A professional industrial-grade web-based controller for 5 Degrees of Freedom (DOF) robotic arm using ESP32. Control your robotic arm through a responsive web interface with real-time updates, position presets, motion recording, and potentiometer support.

📋 Features
Core Functionality
5 Servo Control: Base rotation, shoulder, elbow, wrist pitch, and gripper

Web Interface: Responsive dashboard accessible from any device

Real-time Updates: Live position display and status monitoring

Smooth Motion: Configurable interpolation between positions

Emergency Stop: Instant servo disconnection for safety

Advanced Features
Position Presets: Save and load up to 10 positions with custom names

Motion Recording: Record and playback sequences (up to 150 steps)

Potentiometer Control: Analog control for 4 axes (base, shoulder, elbow, wrist)

Gripper Button: Physical button control for gripper (press open, release close)

Status LEDs: Visual indicators for system states

System Logging: Real-time logging with error/warning categories

Auto Shutdown: Servos detach after 5 minutes of inactivity

Connectivity
WiFi Station Mode: Connect to existing network

AP Mode: Fallback access point mode

mDNS Support: Access via robotarm.local

REST API: Full control via HTTP endpoints

🔧 Hardware Requirements
Components
ESP32 development board

5x Servo motors (standard 50Hz PWM)

4x Potentiometers (10kΩ recommended)

1x Push button (for gripper)

3x Status LEDs (optional)

Power supply (sufficient for servos)

Pin Configuration
Component	GPIO Pin	Notes
Base Servo	25	0-180° rotation
Shoulder Servo	26	0-180° movement
Elbow Servo	27	0-180° movement
Wrist Pitch Servo	14	0-180° movement
Gripper Servo	12	0-90° range
Base Potentiometer	32	ADC input
Shoulder Potentiometer	33	ADC input
Elbow Potentiometer	34	ADC input
Wrist Potentiometer	35	ADC input
Gripper Button	13	Input with pull-up
Green LED	5	Status indicator
Blue LED	18	Recording indicator
Red LED	19	Emergency indicator
Built-in LED	2	System status
📦 Installation
Prerequisites
Arduino IDE or PlatformIO

ESP32 board support installed

Required libraries:

WiFi.h

WebServer.h

ESPmDNS.h

ESP32Servo.h

EEPROM.h

ArduinoJson.h

Setup Steps
Clone the repository

bash
git clone https://github.com/yourusername/esp32-5dof-robot-arm.git
Configure WiFi credentials
Edit the following lines in the code:

cpp
const char* ssid = "YourWiFiSSID";
const char* password = "YourWiFiPassword";
Adjust servo limits (if needed)

cpp
#define GRIPPER_MIN 0
#define GRIPPER_MAX 90
#define WRIST_MIN 0
#define WRIST_MAX 180
Calibrate potentiometers
Adjust ADC min/max values for your specific pots:

cpp
#define POT_BASE_ADC_MIN 200
#define POT_BASE_ADC_MAX 3800
Upload to ESP32

Select correct board and port

Upload the sketch

🚀 Usage
Accessing the Web Interface
Connect to the ESP32

WiFi Mode: Connect to your network, access via http://robotarm.local or the IP shown in Serial Monitor

AP Mode: Connect to WiFi network RobotArm-Controller (password: admin123), then access http://192.168.4.1

Control Panel Overview

https://docs/control-panel.png

Servo Controls: Individual sliders for each servo

Quick Action Buttons: Preset positions for each joint

Special Controls: Dedicated wrist and gripper controls

Recording Panel: Record and playback motion sequences

Presets Grid: Saved positions with visual indicators

System Logs: Real-time activity log

Basic Operations
Manual Control
Use sliders to adjust individual servo positions

Click quick action buttons for common angles

Watch real-time position displays update

Position Presets
Adjust all servos to desired position

Click "SAVE POSITION" button

Enter a name and save

Click any preset card to load that position

Motion Recording
Click "START RECORD" to begin recording

Move servos manually (movements are recorded)

Click "STOP" when finished

Click "PLAY" to replay the sequence

Potentiometer Mode
Toggle "POT MODE" to enable analog control

Web sliders are disabled in this mode

Physical pots control base, shoulder, elbow, and wrist

Emergency Controls
Emergency Stop: Red button or press SPACE/ESC

Home All: Return all servos to home position

Enable/Disable Servos: Toggle power to all servos

API Endpoints
Endpoint	Method	Description
/api/status	GET	Get system status
/api/servo/{1-5}?angle={value}	GET	Move specific servo
/api/emergency	GET	Emergency stop
/api/enable	GET	Enable servos
/api/home	GET	Home all servos
/api/save_preset	POST	Save current position
/api/load_preset/{index}	GET	Load preset
/api/presets	GET	List all presets
/api/record/start	GET	Start recording
/api/record/stop	GET	Stop recording
/api/record/play	GET	Play recording
/api/pot/toggle	GET	Toggle potentiometer mode
/api/logs	GET	Get system logs
⚙️ Configuration
Servo Limits
Adjust these based on your mechanical setup:

cpp
#define SERVO_MIN 0
#define SERVO_MAX 180
#define GRIPPER_MIN 0
#define GRIPPER_MAX 90
Home Positions
Set your preferred home positions:

cpp
#define HOME_BASE 90
#define HOME_SHOULDER 90
#define HOME_ELBOW 90
#define HOME_WRIST 90
#define HOME_GRIPPER 45
Potentiometer Calibration
Adjust ADC ranges for your specific potentiometers:

cpp
#define POT_BASE_ADC_MIN 200
#define POT_BASE_ADC_MAX 3800
// ... similar for other pots
🔧 Troubleshooting
Common Issues
Servos not moving

Check power supply (servos need adequate current)

Verify servo connections

Check if emergency stop is active

Ensure servos are enabled

WiFi connection fails

Check SSID and password

ESP32 will start in AP mode automatically

Connect to RobotArm-Controller network

Potentiometers not responding

Enable pot mode via web interface

Check ADC pin connections

Verify pot mode is not disabled during playback

Recording not working

Ensure servos are enabled

Stop any active playback

Check if storage is full (max 150 steps)

LED Status Indicators
LED	State	Meaning
Green	Solid	System ready, servos enabled
Green	Pulse	Playback step executed
Blue	Solid	Recording active
Blue	Blinking	Playback active
Red	Solid	Emergency stop active
📊 Memory Usage
EEPROM: 2048 bytes allocated

Presets: 10 positions (80 bytes each)

Recording: 150 steps (7 bytes each)

Heap: Real-time monitoring in web interface

🤝 Contributing
Contributions are welcome! Please feel free to submit a Pull Request.
