# Stewart Platform Control & Data Filtering System

**Author:** D.H.A.Tuan

**Python Version:** 3.10+

**MCU:** Arduino Uno

**Last Updated:** 2026-06-19

---

## What is this?

This project is a complete software tool for collecting and analyzing load-cell measurements from a 6-leg Stewart platform. It connects to an Arduino-based measurement system, filters the sensor data, computes platform forces and moments, and displays the results in a desktop GUI.

## What is this used for?

This application is useful for:

- validating and testing Stewart platform force distribution
- monitoring load-cell performance in real time
- comparing raw, filtered, and Kalman-smoothed sensor outputs
- exporting measurement data for further analysis
- learning how to build a robotics data acquisition and visualization system

## Preview


![Stewart Platform GUI screenshot](/asset/image/preview.png)

---

## Project Overview

This repository contains a Python application for real-time acquisition, filtering, visualization, and logging of load-cell data from a 6-leg Stewart platform.

The system reads serial data from an Arduino, applies a filtering pipeline, computes force and moment results, and presents them in a PyQt5 GUI.

### Main capabilities

- Serial connection to Arduino (USB COM port)
- Load-cell data capture for 6 channels
- Low-pass filter and Kalman filter processing
- Force and moment calculation for the Stewart platform
- 2D and 3D plotting with `matplotlib`
- CSV logging and JSON config save/load
- GUI built with Qt Designer `.ui` files

---

## Project structure

- `main.py` — primary application using `StewartPlatformGUI_V3.py`
- `main_v2.py` — alternate GUI entrypoint using `StewartPlatformGUI_V4.py`
- `serial_reader.py` — threaded serial reader and parser
- `StewartPlatformConfig.py` — platform configuration helper
- `StewartPlatformGEN.py` — geometry and platform point generator
- `kalman_filter.py` — Kalman filtering utilities
- `position_estimation.py` — estimation example code
- `StewartPlatformGUI_V3.ui` / `StewartPlatformGUI_V4.ui` — Qt Designer GUI definitions
- `Config/` — JSON configuration files
- `Excel/` — sample measurement CSV files

---

## Requirements

This project requires Python 3.10 or newer and the following packages:

- `pyqt5`
- `pyqt5-tools`
- `matplotlib`
- `numpy`
- `pyserial`
- `pandas`

Optional packages:

- `scipy` (if you want advanced signal processing or additional math utilities)

---

## Installation

Open a terminal in the project root folder and follow these steps.

### 1. Create and activate a virtual environment

Windows PowerShell:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate
```

macOS / Linux:

```bash
python3 -m venv .venv
source .venv/bin/activate
```

### 2. Install required Python packages

```bash
pip install pyqt5 pyqt5-tools matplotlib numpy pyserial pandas
```

If you want optional scientific libraries:

```bash
pip install scipy
```

### 3. Verify Python and pip

```bash
python --version
pip --version
```

If `pyuic5` is not available, install PyQt5 tools:

```bash
pip install pyqt5-tools
```

---

## Regenerating the GUI Python file

If you change a `.ui` file, regenerate the corresponding Python GUI module.

For the V3 GUI:

```bash
pyuic5 -o StewartPlatformGUI_V3.py StewartPlatformGUI_V3.ui
```

For the V4 GUI:

```bash
pyuic5 -o StewartPlatformGUI_V4.py StewartPlatformGUI_V4.ui
```

> Note: Only regenerate the GUI file if you modify the `.ui` file. Otherwise the shipped `.py` files are already usable.

---

## Running the application

### Run the main V3 GUI

```bash
python main.py
```

### Run the V4 GUI

```bash
python main_v2.py
```

### Notes

- Make sure the Arduino is connected before opening the GUI.
- Select the correct COM port and baud rate in the GUI.
- The default baud rate is usually `115200`.

---

## Serial data format

The Arduino must send six load-cell values in a single line.

Expected format:

```text
L: 10.5,9.8,11.0,10.2,9.9,10.7
```

This parser is implemented in `serial_reader.py`.

---

## Configuration

Config files are stored in the `Config/` folder.

Example JSON:

```json
{
  "Rb": 100,
  "Rp": 80,
  "legLength": 120,
  "thetaB": [0, 60, 120, 180, 240, 300],
  "thetaP": [0, 60, 120, 180, 240, 300]
}
```

Use the GUI to load or save a JSON configuration file.

---

## Running from source in development

1. Activate the virtual environment
2. Install dependencies
3. Run `python main.py` or `python main_v2.py`

If you edit the GUI layout, regenerate the `.py` file first.

---

## Troubleshooting

- `ModuleNotFoundError: No module named 'PyQt5'`
  - Activate the virtual environment and reinstall `pyqt5`.
- `pyuic5: command not found`
  - Install `pyqt5-tools` and ensure the virtual environment is active.
- COM port does not appear
  - Reconnect the Arduino, check Windows Device Manager, and select the correct port.
- GUI freezes when serial data arrives
  - `serial_reader.py` should run in a separate thread. Do not block the GUI event loop.

---


