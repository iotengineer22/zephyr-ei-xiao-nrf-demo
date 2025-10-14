# zephyr-ei-xiao-nrf-demo
This repository present Zephyr Demo with Edge Impulse and XIAO nRF54l15/nRF52840 Sense

## Introduction
This repository showcases examples of gesture and voice recognition using the onboard sensors of the **Seeed Studio XIAO nRF52840 Sense** and **XIAO nRF54L15 Sense**. 

The projects are built with the Zephyr RTOS and leverage machine learning models deployed from Edge Impulse.

> **Disclaimer:** This is a personal project created for quick demonstration purposes. Please consider it as a reference, as the code may not be fully polished or production-ready.


## Structure
    .
    ├── raw_data                                        # Training data(IMU_Data, Sound_Data_Link)   
    ├── src                                             # Program and board files 
    │    ├── nrf52840                                   # XIAO nRF52840 Sense
    │    │    ├── imu_dataforwarder_nrf52_12_5hz        # Collecting accel sensor data 12.5Hz        
    │    │    ├── imu_real_inference                    # Gesture Recognition 
    │    │    ├── dmic_inference                        # Voice Recognition
    │    ├── nrf54L15                                   # XIAO nRF54L15 Sense
    │    │    ├── imu_dataforwarder                     # Collecting accel sensor data 104Hz
    │    │    ├── imu_real_inference                    # Gesture Recognition
    │    │    ├── dmic_inference                        # Voice Recognition
    ├── LICENSE
    └── README.md


## Demo Videos

See the examples in action on YouTube:

*   **Gesture Recognition Demo:** [https://youtu.be/FqW0INMxQlg](https://youtu.be/FqW0INMxQlg)
*   **Voice Recognition Demo:** [https://youtu.be/IbxTMiPwFTU](https://youtu.be/IbxTMiPwFTU)

## Overview

The core logic for each example is straightforward:

*   **Voice Recognition:**
    1.  The application captures 1 second of audio data from the onboard microphone.
    2.  It then runs inference on the collected data to classify the sound.
    3.  This process repeats in a continuous loop.

*   **Gesture Recognition:**
    1.  The application captures 2 seconds of data from the 3-axis accelerometer (X, Y, Z) 104Hz.
    2.  It runs inference on the collected data to classify the gesture.
    3.  This process repeats in a continuous loop.

## How to Build and Run

To get this running on your own board, follow these steps:

1.  **Export Model from Edge Impulse:**
    *   Go to your project on [Edge Impulse](https://www.edgeimpulse.com/).
    *   Navigate to the **Deployment** tab.
    *   Select the **C++ library** and click **Build**. This will download a `.zip` file containing the inference SDK and your trained model.

2.  **Copy SDK to Project:**
    *   Unzip the downloaded file.
    *   Copy all the contents (the `edge-impulse-sdk/`, `model-parameters/`, and `tflite-model/` directories) into the `src` folder of this Zephyr project, overwriting the existing files.

3.  **Build and Flash:**
    *   Open your terminal, navigate to the project directory, and run the standard Zephyr build and flash commands for your board. For the XIAO nRF52840 Sense, it would be:

    ```bash
    # Build the application
    west build -p -b xiao_ble/nrf52840/sense

    # Flash it to the device
    west flash -r uf2
    ```
    *For the XIAO nRF54L15 Sense: Please use the nRF Connect for VS Code extension to build the project.

    (This has been verified with version 3.0.1).
