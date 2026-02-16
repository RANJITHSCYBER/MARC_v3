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
Position Presets: Save and load up to 10 positions with custom names

Motion Recording: Record and playback sequences (up to 150 steps)

Potentiometer Control: Analog control for 4 axes (base, shoulder, elbow, wrist)

Gripper Button: Physical button control for gripper (press open, release close)

Status LEDs: Visual indicators for system states

System Logging: Real-time logging with error/warning categories

Auto Shutdown: Servos detach after 5 minutes of inactivity

🌐 Connectivity
WiFi Station Mode: Connect to existing network

AP Mode: Fallback access point mode

mDNS Support: Access via robotarm.local

REST API: Full control via HTTP endpoints

🔧 Hardware Requirements
Required Components
text
┌─────────────────────────────────────┐
│ Component           │ Quantity      │
├─────────────────────┼───────────────┤
│ ESP32 Development   │ 1             │
│ Servo Motors        │ 5             │
│ Potentiometers (10k)│ 4             │
│ Push Button         │ 1             │
│ Status LEDs         │ 3 (optional)  │
│ Power Supply        │ 1 (adequate)  │
└─────────────────────┴───────────────┘
🔌 Pin Configuration
Servo Connections
Servo	GPIO Pin	Range	Function
Base	25	0-180°	Rotation
Shoulder	26	0-180°	Lift/Lower
Elbow	27	0-180°	Extension
Wrist	14	0-180°	Pitch
Gripper	12	0-90°	Open/Close
Sensor & Indicator Connections
Component	GPIO Pin	Type
Base Pot	32	ADC Input
Shoulder Pot	33	ADC Input
Elbow Pot	34	ADC Input
Wrist Pot	35	ADC Input
Gripper Button	13	Digital Input (Pull-up)
Green LED	5	Digital Output
Blue LED	18	Digital Output
Red LED	19	Digital Output
Built-in LED	2	Digital Output
📦 Installation
Prerequisites
bash
# Required Libraries
- WiFi.h
- WebServer.h
- ESPmDNS.h
- ESP32Servo.h
- EEPROM.h
- ArduinoJson.h
