# PENTHUAURA

A Non-Invasive IoT Health Monitoring System with Machine Learning for Advanced Vitals Prediction

PenthuAura is a comprehensive, low-cost Internet of Things (IoT) health monitoring system designed as an academic proof-of-concept for our Bachelor's degree in Communications and Electronics Engineering. It demonstrates the power of combining modern embedded systems, cloud communication strategies, and machine learning to create accessible and intelligent health technology. The system non-invasively captures physiological data and uses a sophisticated machine learning pipeline to predict advanced metrics like cuffless blood pressure, stress levels, and overall health status.

## Key Features

*   **Non-Invasive Sensing:** Utilizes a MAX30102 sensor for photoplethysmography (PPG) and a MAX30205 for high-accuracy temperature sensing.
*   **Advanced Vitals Prediction:** Employs a machine learning pipeline to predict Systolic and Diastolic Blood Pressure (SBP/DBP), user stress levels, and an overall health state from the PPG signal.
*   **Distributed IoT Architecture:** Built on a multi-layered architecture featuring an ESP32-based edge device for data acquisition and a Raspberry Pi backend server for intensive processing.
*   **Robust Communication Pipeline:** Implements a novel communication bridge using ThingSpeak and Ngrok, enabling the edge device to reliably connect to the backend server on any private network.
*   **Signal Standardization:** A critical three-stage signal processing pipeline ensures that data from the low-cost sensor is standardized and comparable to the high-fidelity clinical data used for model training.
*   **Dual User Interface:** Provides both an on-device 16x2 LCD for at-a-glance information and a comprehensive web-based dashboard for configuration and viewing historical data.

## System Architecture

The PenthuAura system is designed with a distributed, multi-layered architecture to balance processing load, power consumption, and responsiveness.

*   **Edge Layer (PenthuAura Device):** The physical device, built around an ESP32 microcontroller, is responsible for data acquisition, user interaction, and data transmission.
*   **Backend Processing Layer (Raspberry Pi Server):** The computational core of the system. A Raspberry Pi runs a Python script that hosts a Flask web server, which receives raw data and executes the entire machine learning pipeline.
*   **Communication & Cloud Layer (ThingSpeak & Ngrok):** This layer acts as an intelligent conduit. Ngrok creates a secure public URL that tunnels to the Flask web server, solving the problem of dynamic IP addresses. ThingSpeak is used as a message broker for the Raspberry Pi to post its public URL and the final processed results.

## Technology Stack

*   **Hardware:** ESP32-WROOM-32, MAX30102, MAX30205, Raspberry Pi, 16x2 LCD.
*   **Firmware (ESP32):** C++/Arduino, ArduinoJSON.
*   **Backend (Raspberry Pi):** Python, Flask, Pandas, Scikit-learn, NeuroKit2, Joblib.
*   **Communication:** HTTP Protocol, ThingSpeak API, Ngrok.

## The Machine Learning Pipeline

The core contribution of this work lies in the sophisticated machine learning pipeline.

*   **Dataset:** The models were trained on the VitalDB clinical dataset, a public database containing high-resolution physiological data, including time-synchronized PPG and invasive Arterial Blood Pressure (ABP) waveforms.
*   **Manual Labeling:** The team undertook a meticulous labeling process, programmatically applying "Stress" and "Health State" labels to each 30-second segment of the VitalDB data based on an extensive review of medical literature.
*   **Signal Standardization:** A crucial three-stage process is applied to both the clinical training data and the live sensor data to ensure a comparable format:
    *   Baseline Correction
    *   Digital Butterworth Bandpass Filtering
    *   Amplitude Scaling
*   **Model Training & Optimization:**
    *   **Feature Extraction:** The NeuroKit2 library was used to extract a comprehensive set of clinically relevant HRV and PPG features.
    *   **Models:** The system uses a Random Forest Regressor for blood pressure, a Random Forest Classifier for stress/health state, and an Isolation Forest for anomaly detection.
    *   **Optimization:** The team addressed class imbalance with SMOTE and tuned hyperparameters using RandomizedSearchCV to achieve high performance, especially in detecting critical health states.

## Getting Started

### Hardware Required

A full Bill of Materials (BOM) can be found in Appendix C of the thesis. Key components include:

*   ESP32-WROOM-32 Dev Kit
*   MAX30102 Module
*   MAX30205 Module
*   Raspberry Pi (Model 4B used in project)
*   16x2 Character LCD
*   Supporting components (resistors, buttons, etc.)

### Setup

*   **Hardware Assembly:** Assemble the prototype circuit as described in Chapter 3 of the thesis.
*   **ESP32 Firmware:** Flash the PenthuAura_Health_Monitor.ino firmware onto the ESP32 using the Arduino IDE. Ensure all required libraries are installed.
*   **Raspberry Pi Backend:**
    *   Set up a Python environment on the Raspberry Pi.
    *   Install all required libraries (`pip install -r requirements.txt`).
    *   Place the trained model files (`.joblib`) and the Flask server script in the appropriate directory.
    *   Configure the system to run the server script automatically on boot using the provided systemd service file (`penthuaura.service`).
*   **Cloud Services:**
    *   Create accounts for ThingSpeak and Ngrok.
    *   Update the server script with your ThingSpeak API keys and Ngrok authentication token.

## Project Team

This project was the result of a dedicated and collaborative team effort for the fulfillment of the Bachelor's degree in Communications and Electronics Engineering.

*   **Diaa Ahmed Hussien:** Project Lead & Machine Learning Engineer
*   **Yehia Mohamed Mahmoud:** Embedded Systems Engineer
*   **Mohanad Tarek Saad:** Backend & Cloud Engineer
*   **Abdulrahman Nageh Abdulrahman:** Frontend Developer & System Testing Lead

Under the supervision of Dr. Omar Mahmoud Sabry.

## Disclaimer

This project is for academic and demonstration purposes only. It is not a certified medical device and should not be used for clinical diagnosis, treatment, or any medical purpose. The readings and predictions are for informational purposes only.

## License

This project is licensed under the MIT License - see the `LICENSE.md` file for details.

