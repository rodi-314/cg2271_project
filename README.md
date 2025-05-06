# CG2271 Project

This group project was completed as part of the CG2271 Real-Time Operating Systems course taught at the National University of Singapore (NUS). The objective of this project is to design and implement a real-time operating system (RTOS)-based robotic car on the ESP32 platform, controlled via a wireless game controller (Wi-Fi/Bluetooth). The system must employ FreeRTOS constructs, including multiple concurrent tasks, inter-task synchronisation mechanisms, and interrupt-driven serial communication. The robotic car is required to perform precise motor control for multidirectional movement, dynamic LED signalling based on motion state, and continuous audio playback through a buzzer. The software architecture must consist of at least four tasks (e.g., motor control, LED control, audio output, and command processing) alongside one interrupt service routine (ISR) for handling asynchronous control inputs.

As part of the assessment, the robotic car will undertake a Challenge Run, during which it must be remotely navigated through a physical maze while fulfilling all functional requirements. The robot's performance will be evaluated based on completion time, successful traversal of obstacles such as ramps, and compliance with specified movement and feedback behaviours.

![image](https://github.com/user-attachments/assets/111e9b83-9232-4821-a9e8-51625b49435a)
