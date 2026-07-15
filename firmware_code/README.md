# Firmware

This directory contains the embedded firmware developed for the AquaSol Smart Irrigation System. The firmware is responsible for sensor data acquisition, LoRa communication, irrigation control, and real-time monitoring across the distributed ESP32 network.

## Directory Structure

- **Master_Node/** – Gateway firmware for data aggregation, cloud communication, dashboard synchronization, and irrigation control.
- **Sensor_Node_1/** – Firmware for monitoring environmental and soil parameters and transmitting data via LoRa.
- **Sensor_Node_2/** – Firmware for distributed field sensing and wireless communication.

## Key Features

- Real-time sensor data acquisition
- Long-range LoRa communication
- Autonomous irrigation control
- OLED-based local monitoring
- Reliable master-slave communication architecture
- Modular and scalable firmware design

> **Note:** Only selected code snippets are included in this repository. The complete firmware is kept private as the project is under active development.
