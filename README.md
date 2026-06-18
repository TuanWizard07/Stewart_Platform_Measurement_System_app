```markdown
# Stewart Platform Control & Data Filtering System
**Author:** D.H.A.Tuan  
**Python Version:** 3.10  
**MCU:** Arduino Uno  
**Last Updated:** (insert date)

---

## 1 — Project Overview
This project provides a **Python + PyQt5 GUI** for real-time acquisition, filtering, visualization, and logging of load-cell data for a **6-leg Stewart platform**.  
The GUI connects to an Arduino-based load-cell interface via serial, processes the data through a filtering pipeline, computes forces & moments, and visualizes results in 2D and 3D using `matplotlib`.

### Key Capabilities
- Serial connection to Arduino (auto COM scan)
- Real-time processing pipeline: **Raw → Scale → LPF → Kalman → Display**
- 6-channel load-cell reading with per-channel filters
- Visual Stewart platform model with color-coded load display
- Configurable geometry (radii, angles, leg length, etc.)
- CSV export and JSON config save/load
- Adjustable plot refresh rates
- GUI built with Qt Designer (`.ui` → `.py` workflow)

**Use case:** Stewart platform force feedback, robotics test rigs, load instrumentation, GUI prototyping.

---

## 2 — System Architecture
```

[Arduino Uno] --(serial)--> [Serial Reader Thread] --> [Parser] --> [Filter Pipeline]
|
+--> [Force/Moment Calc] --> [2D/3D Plots]
|
+--> [CSV Logger] + [Config Loader/Saver]

````

### Component Summary
| Component | Role |
|-----------|------|
| Arduino Uno | Streams six load-cell values via serial text |
| SerialReader | Threaded non-blocking data capture |
| PyQt5 GUI | User controls, plots, filtering toggles |
| Filters | Low-Pass + Kalman (per channel) |
| Geometry module | Computes 3×6 base & platform coordinates |
| CSV Logger | Saves timestamped filtered + raw data |
| Config system | JSON-based load/save of geometry settings |

---

## 3 — Hardware & Requirements
| Item | Specification |
|-------|--------------|
| MCU | Arduino Uno (USB Serial) |
| Serial Format | ASCII, newline-terminated |
| Recommended Baud | 115200 |
| Python | 3.10 documented/tested |
| OS | Windows / macOS / Linux |

---

## 4 — Installation
```bash
# Create virtual environment
python3.10 -m venv venv
source venv/bin/activate    # macOS/Linux
venv\Scripts\activate       # Windows

# Install dependencies
pip install pyqt5 pyqt5-tools matplotlib numpy pyserial
pip install scipy   # optional
````

### Convert Qt Designer `.ui` file

```bash
pyuic5 -o StewartPlatformGUI_V3.py StewartPlatformGUI_V3.ui
```

### Run the GUI

```bash
python main.py
```

---

## 5 — GUI Overview

* **COM Port & Baudrate** selectors (auto refresh every 2 s)
* **Load Cell Display** (raw, LPF, Kalman toggles)
* **Filter Settings** (`alpha` for LPF, `Q/R` for Kalman)
* **Geometry Config** (`Rb`, `Rp`, `legLen`, θ angles)
* **3D Stewart Plot** (color-mapped legs based on load)
* **2D Force/Moment Plots** (Fx, Fy, Fz, Mx, My, Mz)
* **Start/Stop Logging** and **Export CSV**
* **Load/Save Config (JSON)**

Timers:

* 3D redraw: slower, e.g. 2s
* 2D plot update: faster, e.g. 200ms
* COM scan: ~2s interval

---

## 6 — Data Flow & Filtering Pipeline

### **Pipeline Order**

```
Raw → Scale → LPF → Kalman → Display
```

| Stage   | Description                                                  |
| ------- | ------------------------------------------------------------ |
| Raw     | Parsed numbers from Arduino's serial text                    |
| Scale   | Convert ADC counts → Newtons (optional step if needed)       |
| LPF     | Exponential smoothing (`alpha` = 0.0–1.0)                    |
| Kalman  | 1D filter per channel, tuned with `Q` and `R`                |
| Display | Values used for plots, color mapping, CSV, force/moment math |

**Why this order?** LPF knocks down high-frequency noise first, Kalman cleans statistical noise second.

---

## 7 — Low-Pass Filter (LPF)

Formula:

```
y[n] = α*x[n] + (1−α)*y[n−1]
```

* α near `1.0` → reacts quickly
* α near `0.0` → reacts slowly, smoother
* Typical start value: **0.3 – 0.6**

Implementation: simple class storing previous value.

---

## 8 — Kalman Filter (1D per channel)

Standard scalar KF:

```
Predict:
  x_pred = x
  P_pred = P + Q

Update:
  K = P_pred / (P_pred + R)
  x = x_pred + K*(z - x_pred)
  P = (1-K)*P_pred
```

Suggested starting values:

```
Q = 1e-3
R = 1e-3
P_init = 1.0
```

Tuning advice:

* Too slow? → increase `Q`
* Too jumpy? → increase `R`

---

## 9 — Geometry (High-Level Only)

* Base points **Bᵢ** lie in XY plane (`z=0`)
* Platform points **Pᵢ** lie in **offset Z plane**
* Coordinates defined by:

  * Base radius `Rb`, platform radius `Rp`
  * Base angles `θB[]`, platform angles `θP[]` (6 each, degrees)
* Leg vector:
  `uᵢ = (Pᵢ − Bᵢ) / ||Pᵢ − Bᵢ||`
* Force per leg:
  `Fᵢ = loadᵢ * uᵢ`
* Total force = sum of `Fᵢ`
* Total moment = Σ (rᵢ × Fᵢ), with rᵢ relative to chosen origin

*(Full mathematical derivation intentionally excluded.)*

---

## 10 — CSV Logging Format

Example columns:

```
Timestamp (s), L1, L2, L3, L4, L5, L6,
Fx, Fy, Fz, Mx, My, Mz,
Fx_kalman, Fy_kalman, ...
Fx_lpf, Fy_lpf, ...
```

Sample row:

```
12.345, 1.23, 0.98, 1.01, 1.00, 0.99, 1.05, 0.10, -0.05, 5.12, 10.1, -3.5, ...
```

---

## 11 — Config File (JSON)

```json
{
    "Rb": 100,
    "Rp": 80,
    "legLength": 120,
    "thetaB": [0, 60, 120, 180, 240, 300],
    "thetaP": [0, 60, 120, 180, 240, 300]
}
```

---

## 12 — Serial Protocol (Arduino)

**Format:**

```
L: 10.5,9.8,11.0,10.2,9.9,10.7
```

Arduino sample:

```cpp
Serial.begin(115200);
void loop() {
  Serial.print("L: ");
  for (int i=0; i<6; i++) {
    Serial.print(force[i], 2);
    if (i<5) Serial.print(',');
  }
  Serial.println();
  delay(10); // ~100 Hz
}
```

---

## 13 — Troubleshooting

| Issue             | Fix                                                                      |
| ----------------- | ------------------------------------------------------------------------ |
| COM port missing  | Replug USB, check drivers (UNO/CH340), restart GUI                       |
| GUI freezes       | You probably moved serial read into UI thread — move to thread or QTimer |
| Plotting too slow | Lower 3D redraw rate, reduce sample rate, disable antialias              |
| Filter unstable   | Set sane defaults: `alpha=0.5`, `Q=1e-3`, `R=1e-3`                       |
| Units wrong       | Apply scaling **before** LPF/Kalman                                      |

---

## 14 — Repository Structure (Recommended)

```
/main.py
/serial_reader.py
/filters.py
/geometry.py
/StewartPlatformGUI_V3.ui
/StewartPlatformGUI_V3.py   (auto-generated)
/config_examples/*.json
/logs/*.csv
/README.md
```

---

## 15 — License

```
MIT License

Copyright (c) (year) D.H.A.Tuan

Permission is hereby granted, free of charge, to any person obtaining a copy
...
```

---

## 16 — Author

**D.H.A.Tuan**
(Insert contact / GitHub username if desired)

---

## 17 — Quick Start Checklist

✅ Install Python 3.10
✅ `pip install` dependencies
✅ Convert `.ui` → `.py` using `pyuic5`
✅ Upload Arduino sketch
✅ Connect COM & click **Start Streaming**
✅ Enable filters and visualizations
✅ Log → Export CSV when finished

---

### Final Note

Want reliable real-time plotting? Keep serial I/O threaded, avoid blocking the GUI, and don’t try to redraw 3D views faster than ~1–2 Hz.
Tune filters with **real data**, not theory. Hardware noise always wins.

```

---

If you want:

✅ `requirements.txt`  
✅ Arduino `.ino` example file  
✅ `config_example.json` template  

Just say the word and I’ll generate them.
```
