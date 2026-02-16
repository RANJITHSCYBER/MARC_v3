5-DOF Robotic Arm Controller for ESP32
<p align="center"> <img src="https://img.shields.io/badge/version-3.0-blue?style=for-the-badge" /> <img src="https://img.shields.io/badge/platform-ESP32-green?style=for-the-badge" /> <img src="https://img.shields.io/badge/servos-5%20DOF-orange?style=for-the-badge" /> <img src="https://img.shields.io/badge/license-MIT-yellow?style=for-the-badge" /> </p><p align="center"> A professional industrial-grade web-based controller for 5 Degrees of Freedom (DOF) robotic arm using ESP32. Control your robotic arm through a responsive web interface with real-time updates, position presets, motion recording, and potentiometer support. </p>
📋 Table of Contents
Features

Hardware Requirements

Pin Configuration

Installation

Usage

API Reference

Configuration

Troubleshooting

Contributing

License

✨ Features
🎮 Core Functionality
Feature	Description
5 Servo Control	Base rotation, shoulder, elbow, wrist pitch, and gripper
Web Interface	Responsive dashboard accessible from any device
Real-time Updates	Live position display and status monitoring
Smooth Motion	Configurable interpolation between positions
Emergency Stop	Instant servo disconnection for safety
🚀 Advanced Features
Feature	Description
Position Presets	Save and load up to 10 positions with custom names
Motion Recording	Record and playback sequences (up to 150 steps)
Potentiometer Control	Analog control for 4 axes (base, shoulder, elbow, wrist)
Gripper Button	Physical button control for gripper (press open, release close)
Status LEDs	Visual indicators for system states
System Logging	Real-time logging with error/warning categories
Auto Shutdown	Servos detach after 5 minutes of inactivity
🌐 Connectivity
Feature	Description
WiFi Station Mode	Connect to existing network
AP Mode	Fallback access point mode
mDNS Support	Access via robotarm.local
REST API	Full control via HTTP endpoints
🔧 Hardware Requirements
Required Components
Component	Specification	Quantity	Purpose
ESP32 Development Board	Any variant (NodeMCU-32S, DOIT, etc.)	1	Main controller
Servo Motors	Standard 50Hz PWM (SG90, MG90, MG996R)	5	Joint actuation
Potentiometers	10kΩ linear taper	4	Analog position control
Push Button	Momentary, normally open	1	Gripper control
Status LEDs	5mm, any color	3	Visual indicators
Current Limiting Resistors	220Ω - 330Ω	3	For LEDs
Power Supply	5V-6V, 5A+ recommended	1	Servo power
Voltage Regulator	LM2596 or similar (optional)	1	If using higher voltage
Capacitors	1000µF and 100µF	2 each	Power smoothing
Jumper Wires	Male-to-female, male-to-male	As needed	Connections
Breadboard	400/800 points	1	Prototyping
Mounting Hardware	Screws, brackets, etc.	As needed	Mechanical assembly
Optional Components
Component	Specification	Quantity	Purpose
OLED Display	128x64, I2C	1	Local status display
Buzzer	5V piezo	1	Audio feedback
Level Shifter	3.3V to 5V	1	If using 5V logic devices
Enclosure	Custom 3D printed	1	Protection
Heat Sinks	For motor drivers	5	Thermal management
Power Requirements
Component	Voltage	Current (per unit)	Total Current
ESP32	3.3V/5V	80mA	80mA
Servo Motors	5V-6V	500mA-1A (stall)	2.5A-5A
LEDs	2V-3V	20mA	60mA
Total	5V-6V		~3A-6A
🔌 Pin Configuration
Servo Connections
Servo	GPIO Pin	PWM Channel	Range	Function
Base Rotation	25	0	0-180°	Rotates arm base
Shoulder	26	1	0-180°	Lifts/lowers arm
Elbow	27	2	0-180°	Extends/retracts
Wrist Pitch	14	3	0-180°	Wrist up/down
Gripper	12	4	0-90°	Open/close
Sensor & Input Connections
Component	GPIO Pin	ADC Channel	Type	Function
Base Potentiometer	32	ADC1_CH4	Analog Input	Base position control
Shoulder Potentiometer	33	ADC1_CH5	Analog Input	Shoulder position control
Elbow Potentiometer	34	ADC1_CH6	Analog Input	Elbow position control
Wrist Potentiometer	35	ADC1_CH7	Analog Input	Wrist position control
Gripper Button	13	-	Digital Input (Pull-up)	Gripper control
Indicator Connections
Component	GPIO Pin	Type	Function	Current Limiting
Green LED	5	Digital Output	System ready	220Ω resistor
Blue LED	18	Digital Output	Recording active	220Ω resistor
Red LED	19	Digital Output	Emergency stop	220Ω resistor
Built-in LED	2	Digital Output	Status	Onboard resistor
Power Connections
Connection	From	To	Wire Gauge
Main Power	Power Supply (5V)	ESP32 VIN	18-22 AWG
Servo Power	Power Supply (5V)	Servo VCC (all)	18-22 AWG
Ground	Power Supply GND	ESP32 GND, Servo GND	18-22 AWG
Signal	ESP32 GPIO	Servo Signal	24-26 AWG
📦 Installation
Prerequisites
bash
# Required Libraries (install via Arduino Library Manager)
- WiFi.h (built-in)
- WebServer.h (built-in)
- ESPmDNS.h (built-in)
- ESP32Servo.h (by Kevin Harrington)
- EEPROM.h (built-in)
- ArduinoJson.h (by Benoit Blanchon)
Software Setup
Clone the repository

bash
git clone https://github.com/yourusername/esp32-5dof-robot-arm.git
cd esp32-5dof-robot-arm
Configure WiFi credentials

cpp
// Edit these lines in the main .ino file
const char* ssid = "YourWiFiSSID";
const char* password = "YourWiFiPassword";
Install required libraries

bash
# Using Arduino CLI
arduino-cli lib install "ESP32Servo"
arduino-cli lib install "ArduinoJson"
Compile and upload

bash
# Using Arduino IDE
- Select board: "ESP32 Dev Module"
- Select correct COM port
- Click Upload button

# Using PlatformIO
platformio run --target upload
Hardware Assembly
Prepare the power supply

text
┌─────────────────────┐
│  Power Supply (5V)  │
└─────────┬───────────┘
          │
  ┌───────┴───────┐
  │               │
[GND]          [VCC]
  │               │
  └───────┬───────┘
          │
  ┌───────┴───────┐
  │   ESP32 VIN   │
  │   Servo VCC   │
  └───────────────┘
Connect servos in parallel

text
Power Supply (+5V) ─┬─ Servo1 (Red)
                    ├─ Servo2 (Red)
                    ├─ Servo3 (Red)
                    ├─ Servo4 (Red)
                    └─ Servo5 (Red)

Power Supply (GND) ─┬─ Servo1 (Brown)
                    ├─ Servo2 (Brown)
                    ├─ Servo3 (Brown)
                    ├─ Servo4 (Brown)
                    └─ Servo5 (Brown)
Add decoupling capacitors

text
Power (+5V) ─────┬──── Capacitor (+)
                 │     1000µF
Power (GND) ─────┴──── Capacitor (-)
🚀 Usage
Access Methods
Mode	Connection Method	SSID	Password	Access URL
WiFi Mode	Connect to your network	Your Network SSID	Your Network Password	http://robotarm.local or DHCP IP
AP Mode	Connect directly to ESP32	RobotArm-Controller	admin123	http://192.168.4.1
Control Panel Overview
text
┌─────────────────────────────────────────────────────────────┐
│                       HEADER BAR                             │
├─────────────────────────────────────────────────────────────┤
│  [🤖] 5-DOF Robotic Arm    ● WiFi Connected    ● Servos ON  │
└─────────────────────────────────────────────────────────────┘

┌─────────────────┬─────────────────┬─────────────────┐
│    BASE         │    SHOULDER     │     ELBOW       │
│    [90°]        │    [90°]        │    [90°]        │
│  ┌─────────┐    │  ┌─────────┐    │  ┌─────────┐    │
│  │=========│    │  │=========│    │  │=========│    │
│  └─────────┘    │  └─────────┘    │  └─────────┘    │
│  0°    180°     │  0°    180°     │  0°    180°     │
│ [0°] [90°][180°]│ [0°] [90°][180°]│ [0°] [90°][180°]│
└─────────────────┴─────────────────┴─────────────────┘

┌─────────────────┬─────────────────┐
│     WRIST       │    GRIPPER      │
│    [90°]        │    [45°]        │
│  ┌─────────┐    │  ┌─────────┐    │
│  │=========│    │  │=====    │    │
│  └─────────┘    │  └─────────┘    │
│  0°    180°     │  0°     90°     │
│ [↓] [⏏] [↑]     │ [O] [⏏] [C]     │
└─────────────────┴─────────────────┘

┌─────────────────────────────────────┐
│        CONTROL BUTTONS               │
├─────────────────────────────────────┤
│ [🔴 EMERGENCY]  [🏠 HOME]  [⚡ ENABLE]│
│ [📝 POT MODE]   [💾 SAVE]            │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│        RECORDING CONTROLS            │
├─────────────────────────────────────┤
│ [● RECORD] [■ STOP] [▶ PLAY] [🗑 CLEAR]│
│ Status: Idle    Steps: 0/150         │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│        POSITION PRESETS              │
├─────────────────────────────────────┤
│ ┌────────┐ ┌────────┐ ┌────────┐   │
│ │ Home   │ │ Pick   │ │ Place  │   │
│ │ 90-90- │ │ 45-120-│ │ 135-60-│   │
│ │ 90-90- │ │ 90-90- │ │ 90-90- │   │
│ │ 45     │ │ 0      │ │ 90     │   │
│ └────────┘ └────────┘ └────────┘   │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│        SYSTEM LOGS                   │
├─────────────────────────────────────┤
│ [12:34:56] System ready              │
│ [12:35:01] WiFi connected            │
│ [12:35:10] Servos initialized        │
└─────────────────────────────────────┘
Quick Start Guide
Step	Action	Expected Result
1	Power on ESP32	Green LED solid, connect to web interface
2	Click "ENABLE SERVOS"	Servos energize, hold position
3	Test each joint with sliders	Servos move smoothly
4	Click "HOME"	All servos return to 90°
5	Adjust gripper with button	Gripper opens/closes
6	Save a position	Preset appears in grid
📡 API Reference
Base Endpoints
Method	Endpoint	Description	Response Format
GET	/	Web interface	HTML
GET	/api/status	System status	JSON
GET	/api/logs	System logs	JSON
Servo Control
Method	Endpoint	Parameters	Description
GET	/api/servo/1	angle=0-180	Control base
GET	/api/servo/2	angle=0-180	Control shoulder
GET	/api/servo/3	angle=0-180	Control elbow
GET	/api/servo/4	angle=0-180	Control wrist
GET	/api/servo/5	angle=0-90	Control gripper
POST	/api/servos	JSON body	Set all servos
System Control
Method	Endpoint	Description
GET	/api/emergency	Emergency stop
GET	/api/enable	Enable servos
GET	/api/home	Home all servos
Preset Management
Method	Endpoint	Description
POST	/api/save_preset	Save current position
GET	/api/load_preset/{index}	Load preset (0-9)
GET	/api/presets	List all presets
GET	/api/clear_presets	Delete all presets
Recording Functions
Method	Endpoint	Description
GET	/api/record/start	Start recording
GET	/api/record/stop	Stop recording
GET	/api/record/play	Play recording
GET	/api/record/stop_playback	Stop playback
GET	/api/record/clear	Clear recording
Special Functions
Method	Endpoint	Description
GET	/api/pot/toggle	Toggle potentiometer mode
GET	/api/gripper/open	Fully open gripper
GET	/api/gripper/close	Fully close gripper
GET	/api/gripper/home	Gripper to home
GET	/api/wrist/up	Wrist up (180°)
GET	/api/wrist/down	Wrist down (0°)
GET	/api/wrist/home	Wrist to home (90°)
Example API Calls
javascript
// Get system status
fetch('/api/status')
  .then(response => response.json())
  .then(data => console.log(data));

// Move base to 90°
fetch('/api/servo/1?angle=90')
  .then(response => response.json())
  .then(data => console.log(data));

// Save current position
fetch('/api/save_preset', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    name: 'Pick Position',
    servo1: 45,
    servo2: 120,
    servo3: 90,
    servo4: 90,
    servo5: 0
  })
});
⚙️ Configuration
Servo Limits
cpp
// Edit these values in configuration section
#define SERVO_MIN       0     // Minimum angle for base/shoulder/elbow
#define SERVO_MAX       180   // Maximum angle for base/shoulder/elbow
#define GRIPPER_MIN     0     // Gripper fully open
#define GRIPPER_MAX     90    // Gripper fully closed
#define WRIST_MIN       0     // Wrist down
#define WRIST_MAX       180   // Wrist up
Home Positions
cpp
// Set your preferred home positions
#define HOME_BASE       90    // Center position
#define HOME_SHOULDER   90    // Level position
#define HOME_ELBOW      90    // Right angle
#define HOME_WRIST      90    // Level position
#define HOME_GRIPPER    45    // Half open
Potentiometer Calibration
cpp
// ADC range calibration (0-4095)
#define POT_BASE_ADC_MIN    200    // Minimum ADC value for base pot
#define POT_BASE_ADC_MAX    3800   // Maximum ADC value for base pot
#define POT_SHOULDER_ADC_MIN 200   // Minimum ADC value for shoulder pot
#define POT_SHOULDER_ADC_MAX 3800  // Maximum ADC value for shoulder pot
#define POT_ELBOW_ADC_MIN   200    // Minimum ADC value for elbow pot
#define POT_ELBOW_ADC_MAX   3800   // Maximum ADC value for elbow pot
#define POT_WRIST_ADC_MIN   200    // Minimum ADC value for wrist pot
#define POT_WRIST_ADC_MAX   3800   // Maximum ADC value for wrist pot

// Potentiometer behavior
#define POT_DEADBAND        2      // Degrees of tolerance (prevents jitter)
#define POT_SAMPLES         5      // Number of ADC samples per read
#define POT_FILTER_ALPHA    0.25f  // Smoothing factor (0-1)
#define POT_READ_INTERVAL   50     // ms between pot reads
Timing Configuration
cpp
#define SERVO_UPDATE_INTERVAL   20      // ms between servo updates
#define INACTIVITY_TIMEOUT      300000  // 5 minutes (ms)
#define BUTTON_DEBOUNCE_MS      30      // Button debounce time
#define BLUE_BLINK_INTERVAL_MS  1000    // LED blink interval
#define GREEN_STEP_PULSE_MS     120     // Green LED pulse duration
Memory Allocation
cpp
#define EEPROM_SIZE         2048    // Total EEPROM bytes
#define MAX_PRESETS         10      // Number of preset slots
#define PRESET_SIZE         80      // Bytes per preset
#define RECORD_MAX_STEPS    150     // Maximum recording steps
#define RECORD_STEP_SIZE    7       // Bytes per recorded step
🔍 Troubleshooting
Common Issues
Issue	Possible Cause	Solution
Servos not moving	Power supply inadequate	Use 5V 5A+ supply, add capacitors
Emergency stop active	Click "ENABLE SERVOS" or reset
Loose connections	Check all servo wires
PWM channel conflict	Reassign channels in code
| WiFi connection fails | Wrong credentials | Verify SSID/password in code |
| | Router issues | ESP32 will auto-start AP mode |
| | Signal too weak | Move ESP32 closer to router |
| | Firewall blocking | Allow port 80 on network |

| Potentiometers not working | Pot mode disabled | Click "POT MODE" button |
| | Wrong ADC pins | Verify connections (32-35) |
| | ADC range wrong | Recalibrate min/max values |
| | Playback active | Stop playback first |

| Recording issues | Storage full | Clear recording (max 150 steps) |
| | Playback active | Stop playback to record |
| | Servos disabled | Enable servos first |
| | Memory corruption | Clear EEPROM and reset |

| Web interface slow | Network congestion | Check WiFi signal |
| | Browser cache | Clear cache, refresh |
| | Too many clients | Limit simultaneous connections |
| | ESP32 overload | Reduce update frequency |

LED Status Reference
LED Pattern	State	Meaning	Action Required
🟢 Green Solid	System Ready	Normal operation	None
🟢 Green Pulse	Playback Step	Executing motion	None
🔵 Blue Solid	Recording Active	Capturing motion	Stop recording when done
🔵 Blue Blinking	Playback Active	Replaying motion	Can stop if needed
🔴 Red Solid	Emergency Stop	Safety active	Click "ENABLE SERVOS"
🔴 Red Blinking	Error	System fault	Check logs, reset
Diagnostic Commands
cpp
// Add to serial monitor for debugging
void diagnosticMode() {
  Serial.println("=== DIAGNOSTIC MODE ===");
  Serial.print("Free Heap: "); Serial.println(ESP.getFreeHeap());
  Serial.print("WiFi RSSI: "); Serial.println(WiFi.RSSI());
  Serial.print("Uptime: "); Serial.println(millis()/1000);
  
  // Test each servo
  for(int i=1; i<=5; i++) {
    moveServo(i, 90);
    delay(500);
    moveServo(i, 0);
    delay(500);
    moveServo(i, 180);
    delay(500);
    moveServo(i, 90);
  }
}
Hardware Debugging
Test Point	Expected Reading	If Different
ESP32 VIN	5V DC	Check power supply
Servo VCC	5V DC	Check power distribution
GPIO 25-27,14,12	3.3V pulse	Check servo connections
ADC Pins 32-35	0-3.3V variable	Check pot connections
Button Pin 13	HIGH (idle), LOW (pressed)	Check pull-up resistor
📊 Memory Specifications
Storage Allocation
Memory Region	Address Range	Size	Purpose
EEPROM	0x0000-0x07FF	2048 bytes	Persistent storage
- Init Flag	0x0000	1 byte	EEPROM initialized?
- Presets	0x0001-0x031F	800 bytes	10 presets × 80 bytes
- Recording Meta	0x0320-0x0321	2 bytes	Recording count
- Recording Data	0x0322-0x07FF	1246 bytes	150 steps × 7 bytes
Preset Structure (80 bytes)
Field	Offset	Size	Description
Name	0	32 bytes	Preset name (null-terminated)
Servo 1	32	1 byte	Base angle (0-180)
Servo 2	33	1 byte	Shoulder angle (0-180)
Servo 3	34	1 byte	Elbow angle (0-180)
Servo 4	35	1 byte	Wrist angle (0-180)
Servo 5	36	1 byte	Gripper angle (0-90)
Speed	37	1 byte	Movement speed (1-100)
Reserved	38-79	42 bytes	Future use
Recording Step Structure (7 bytes)
Field	Offset	Size	Description
Servo 1	0	1 byte	Base angle
Servo 2	1	1 byte	Shoulder angle
Servo 3	2	1 byte	Elbow angle
Servo 4	3	1 byte	Wrist angle
Servo 5	4	1 byte	Gripper angle
Delay MSB	5	1 byte	Delay high byte
Delay LSB	6	1 byte	Delay low byte
🤝 Contributing
Development Workflow
bash
1. Fork the repository
2. Create feature branch
   git checkout -b feature/AmazingFeature
3. Commit changes
   git commit -m 'Add AmazingFeature'
4. Push to branch
   git push origin feature/AmazingFeature
5. Open Pull Request
Coding Standards
Category	Standard
Indentation	2 spaces
Naming Convention	camelCase for variables, PascalCase for functions
Comments	Doxygen style for functions, inline for complex logic
Maximum Line Length	80 characters
Error Handling	Always check return values, provide meaningful messages
Pull Request Checklist
Code compiles without errors

Tested on actual hardware

Updated documentation

Added comments for new functions

Followed coding standards

No debug code left behind

📝 License
text
MIT License

Copyright (c) 2024 [Your Name]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
👏 Acknowledgments
Libraries
ESP32Servo by Kevin Harrington

ArduinoJson by Benoit Blanchon

Font Awesome for icons

Inspiration
Industrial robot controllers

Open source robotics community

ESP32 development community

Contributors
Your Name

Contributor Name

📞 Support & Contact
Channel	Contact Info
GitHub Issues	github.com/yourusername/esp32-5dof-robot-arm/issues
Email	your.email@example.com
Discord	Join our server
Wiki	Project Wiki
📊 Version History
Version	Date	Changes
3.0	2024	Current release - 5 DOF support, recording, pot mode
2.0	2023	Added presets, web interface redesign
1.0	2022	Initial release - Basic servo control
<p align="center"> <strong>Made with ❤️ for robotics enthusiasts</strong> <br> <br> <a href="https://github.com/yourusername/esp32-5dof-robot-arm/issues">🐛 Report Bug</a> • <a href="https://github.com/yourusername/esp32-5dof-robot-arm/issues">✨ Request Feature</a> • <a href="https://github.com/yourusername/esp32-5dof-robot-arm/wiki">📚 Documentation</a> </p><p align="center"> <img src="https://img.shields.io/github/stars/yourusername/esp32-5dof-robot-arm?style=social" /> <img src="https://img.shields.io/github/forks/yourusername/esp32-5dof-robot-arm?style=social" /> <img src="https://img.shields.io/github/watchers/yourusername/esp32-5dof-robot-arm?style=social" /> </p>
