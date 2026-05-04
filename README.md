# Driver Drowsiness Detection System using Heart Rate and Postural Changes

## Overview
This project develops a driver drowsiness detection system using physiological and motion data. The system combines heart rate signals and postural changes to classify driver fatigue levels using a Fuzzy Inference System.

---

## System Overview

![System Diagram](images/system_overview.png)

*Overall functional system diagram for driver drowsiness detection.*

---

## Hardware & Instrumentation

![Device](images/device.jpg)

*Prototype device for acquiring ECG and postural data.*

---

## User Interface

![GUI](images/gui.png)

*Graphical user interface for monitoring and visualization.*

---

## Data Acquisition

### ECG Signal Acquisition
![ECG Diagram](images/ecg_diagram.png)

*Acquisition of heart rate signals using ECG sensor.*

---

### Postural Data Acquisition
![Postural Diagram](images/postural_diagram.png)

*Measurement of head movement and postural changes.*

---

## Hardware Design

![Schematic](images/schematic.png)

*Circuit schematic of the embedded system including ECG sensor and motion sensors.*

## Features

### Physiological Features
- Heart Rate (BPM)

### Postural Features
- Head angle
- Angular velocity
- Nodding duration

---

## Methodology

- Signal acquisition from ECG and motion sensors
- Feature extraction from physiological and postural data
- Classification using **Fuzzy Inference System (FIS)**

---

## Key Findings

- Heart rate tends to decrease during drowsiness
- Postural angle and nodding duration increase
- Angular velocity decreases as fatigue increases

---

## Performance

- Achieved overall accuracy of **87.14%**

---

## Technologies
- Embedded System (STM32)
- Signal Processing
- ECG Sensor
- Accelerometer & Gyroscope
- Fuzzy Logic (FIS)

---

## Project Structure
- `embedded/` – Embedded system code  
- `interface/` – GUI application  
- `hardware/` – PCB and device design  
- `images/` – Diagrams and visualization assets  

---

## Author
Afan Ghafar
