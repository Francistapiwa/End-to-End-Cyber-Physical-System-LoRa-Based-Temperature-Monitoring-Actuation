# End-to-End-Cyber-Physical-System-LoRa-Based-Temperature-Monitoring-Actuation

Overview
This project demonstrates a fully functional cyber-physical system where an ESP32 microcontroller (equipped with a BMP280 temperature sensor) wirelessly transmits real-time temperature data to a PC-based application via LoRa radio. The PC application analyzes the temperature trend and sends actuation commands back to the ESP32 to control its built-in LED (on if temperature is rising, off otherwise).

The system emphasizes:

Real-time task scheduling using FreeRTOS (3 parallel tasks).

Wireless communication (LoRa) for sensor-to-PC and PC-to-actuator links.

Thread-safe resource sharing via FreeRTOS primitives (queues, mutexes).

Trend analysis (PC app determines temperature slope).

Technical Implementation
ESP32 Firmware (Written in C/C++)
FreeRTOS Tasks:

Temperature Sensing Task: Reads BMP280 sensor every 500ms, stores data in a shared queue.

LoRa TX/RX Task: Transmits temperature data to PC; listens for LED commands.

LED Control Task: Processes commands from PC to toggle LED (GPIO).

Concurrency Management:

Mutexes protect shared resources (e.g., sensor data during LoRa transmission).

Queues pass temperature readings and commands between tasks.

Hardware:

BMP280 (I2C) for temperature sensing.

SX1276 LoRa module (SPI) for wireless communication.

Built-in LED (GPIO) for actuation.

PC Application (Python/C++/Java)
Receives LoRa packets, logs temperature trends.

Decision Logic: Sends ON/OFF command based on temperature derivative (ΔT/Δt).

Console Output: Displays live temperature + last sent command.
