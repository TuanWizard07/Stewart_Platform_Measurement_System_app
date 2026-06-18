import sys
import re
import json
import csv
import time
from datetime import datetime

import numpy as np
import serial.tools.list_ports

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QFileDialog, QVBoxLayout, QComboBox,
    QMessageBox, QSizePolicy
)
from PyQt5.QtCore import QTimer

from StewartPlatformGUI_V3 import Ui_MainWindow
from serial_reader import SerialReader
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D
## Functions for other modules
def parse_theta(text):
    return list(map(float, re.findall(r"[-+]?\d*\.\d+|\d+", text)))
def polar_to_cartesian(radius, angles_deg):
    angles_rad = np.radians(angles_deg)
    x = radius * np.cos(angles_rad)
    y = radius * np.sin(angles_rad)
    return np.vstack((x, y))  # 2xN
def build_geometry_from_config(Rb, Rp, thetaB_deg, thetaP_deg, legLength):
    """
    Return BasePoint, PlatformPoint (3x6 arrays)
    Base at z=0; Platform placed at z=calculated to make legLength satisfied
    """
    # 2D coordinates
    B_xy = polar_to_cartesian(Rb, thetaB_deg)  # 2x6
    P_xy = polar_to_cartesian(Rp, thetaP_deg)  # 2x6

    dx = P_xy[0] - B_xy[0]
    dy = P_xy[1] - B_xy[1]

    # Leg projected distances in XY plane
    dxy2 = dx**2 + dy**2
    # Compute platform height needed to satisfy leg length
    dz = np.sqrt(np.abs(legLength**2 - dxy2))  # avoid sqrt(negative)
    z_offset = np.mean(dz)
    # Final 3D points
    B = np.vstack((B_xy, np.zeros(6)))
    P = np.vstack((P_xy, np.full(6, dz)))
    # print("platform points:", P)
    # print("base points:", B)
    return B, P
def computeForceMoment(loadcells, B, P):
    total_force = np.zeros(3)
    total_moment = np.zeros(3)
    center = np.mean(B, axis=1)

    for i in range(6):
        bi = B[:, i]
        pi = P[:, i]
        direction = pi - bi
        unit_vector = direction / np.linalg.norm(direction)
        fi = loadcells[i] * unit_vector
        ri = bi - center
        mi = np.cross(ri, fi)
        total_force += fi
        total_moment += mi
    
    return {
        'Fx': total_force[0],
        'Fy': total_force[1],
        'Fz': total_force[2],
        'Mx': total_moment[0],
        'My': total_moment[1],
        'Mz': total_moment[2]
    }
class KalmanFilter1D:
    def __init__(self, x_init=0.0, P_init=1.0, Q=1e-3, R=0.001):
        """
        Stateful 1D Kalman Filter for real-time data.
        Args:
            x_init: Initial estimate
            P_init: Initial error covariance
            Q: Process noise covariance (higher = trust model more)
            R: Measurement noise covariance (higher = trust sensor less)
        """
        self.x = x_init
        self.P = P_init
        self.Q = Q
        self.R = R

    def update(self, z):
        """
        Update filter with a new measurement.
        Args:
            z: New sensor measurement
        Returns:
            Filtered estimate
        """
        # Predict
        x_pred = self.x
        P_pred = self.P + self.Q

        # Update
        K = P_pred / (P_pred + self.R)
        self.x = x_pred + K * (z - x_pred)
        self.P = (1 - K) * P_pred

        return self.x
class LowPassFilter:
    def __init__(self, alpha=0.5):
        self.alpha = alpha
        self.last_value = 0.0
    def update(self, input_value):
        self.last_value = self.alpha * input_value + (1 - self.alpha) * self.last_value
        return self.last_value
## Main Application Class
import numpy as np
import pandas as pd

from PyQt5.QtWidgets import (
    QWidget, QPushButton, QFileDialog,
    QVBoxLayout, QLabel
)

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.patches import Ellipse

class StewartApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.reader = None
        
        # Force and moment logs
        self.raw_data_log = []  # stores (time, lc1, lc2, lc3, lc4, lc5, lc6)
        self.force_log = []   # stores (time, Fx, Fy, Fz)
        self.moment_log = []  # stores (time, Mx, My, Mz)
        self.kalman_force_log = []   # stores (time, Fx, Fy, Fz)
        self.kalman_moment_log = []  # stores (time, Mx, My, Mz)
        self.filtered_2_force_log = []   # stores (time, Fx, Fy, Fz)
        self.filtered_2_moment_log = []  # stores (time, Mx, My, Mz)
        
        # time_data will store timestamps for plotting
        self.time_data = []   # stores time values for plotting
        self.start_time = None  # to track logging start time
        self.logging_enabled = False
        self.plotting_enabled = False
        self.log_data = []  # Will store (timestamp, Fx, Fy, Fz, Mx, My, Mz
        self.raw_data = {
            'lc1': [], 'lc2': [], 'lc3': [], 
            'lc4': [], 'lc5': [], 'lc6': [],
            'fx': [], 'fy': [], 'fz': [],
            'mx': [], 'my': [], 'mz': []
        }  # Will store raw data from serial
        self.KF_data = {
            'lc1': [], 'lc2': [], 'lc3': [], 
            'lc4': [], 'lc5': [], 'lc6': [],
            'fx': [], 'fy': [], 'fz': [],
            'mx': [], 'my': [], 'mz': []
        }
        self.filtered_2_data = {
            'lc1': [], 'lc2': [], 'lc3': [], 
            'lc4': [], 'lc5': [], 'lc6': [],
            'fx': [], 'fy': [], 'fz': [],
            'mx': [], 'my': [], 'mz': []
        }
        ## Kalman Filters
        self.kalman_filters = {
            'lc1': KalmanFilter1D(),
            'lc2': KalmanFilter1D(),
            'lc3': KalmanFilter1D(),
            'lc4': KalmanFilter1D(),
            'lc5': KalmanFilter1D(),
            'lc6': KalmanFilter1D(),
        }
        self.lowpass_filters = {
            'lc1': LowPassFilter(),
            'lc2': LowPassFilter(),
            'lc3': LowPassFilter(),
            'lc4': LowPassFilter(),
            'lc5': LowPassFilter(),
            'lc6': LowPassFilter(),
        }
        
        self.plot_limit = 50  # Limit to last N points for smoother plotting
        ## Initialize SERIAL READER
        self.init_com_ports()
        self.init_baudrate_options()

        # CHECKBOXES
        self.ui.checkFx.setChecked(True)
        self.ui.checkFy.setChecked(True)
        self.ui.checkFz.setChecked(True)
        self.ui.checkMx.setChecked(True)
        self.ui.checkMy.setChecked(True)
        self.ui.checkMz.setChecked(True)
        self.ui.radioKF.setChecked(True)
        self.ui.checkKF.setChecked(True)
        self.ui.checkLPF.setChecked(True)

        self.com_timer = QTimer()
        self.com_timer.timeout.connect(self.init_com_ports)
        self.com_timer.start(2000)  # Check every 2 seconds

        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.plot_stewart)
        self.plot_timer.start(2000)  # Check every 2 seconds
        
        self.plot_2D_timer = QTimer()
        self.plot_2D_timer.timeout.connect(self.update_force_moment_plots_frequent)
        self.plot_2D_timer.start(200)  # Check every 2 seconds
        
        
        ## Initialize plotting
        self.force_figures = {}
        self.force_axes = {}
        self.force_canvases = {}
        self.plot_2D_setup()
        self.plot_3D_setup()
        ## BUTTON FUNCTIONS
        self.apply_config()
        self.ui.btnConnect.clicked.connect(self.toggle_connection)
        self.ui.btnSaveConfig.clicked.connect(self.save_config)
        self.ui.btnLoadConfig.clicked.connect(self.load_config)
        self.ui.btnApplyConfig.clicked.connect(self.apply_config)
        self.ui.btnStartLog.clicked.connect(self.start_logging)
        self.ui.btnStopLog.clicked.connect(self.stop_logging)
        self.ui.btnExportCSV.clicked.connect(self.export_csv)
        # CHECK BOXES
        self.ui.checkFx.stateChanged.connect(self.update_force_plot)
        self.ui.checkFy.stateChanged.connect(self.update_force_plot)
        self.ui.checkFz.stateChanged.connect(self.update_force_plot)
        self.ui.checkMx.stateChanged.connect(self.update_moment_plot)
        self.ui.checkMy.stateChanged.connect(self.update_moment_plot)
        self.ui.checkMz.stateChanged.connect(self.update_moment_plot)
        self.ui.checkKF.stateChanged.connect(self.update_force_plot)
        self.ui.checkKF.stateChanged.connect(self.update_moment_plot)
        self.ui.checkLPF.stateChanged.connect(self.update_force_plot)
        self.ui.checkLPF.stateChanged.connect(self.update_moment_plot)
        # BUTTON CONNECTIONS
        self.ui.btnSetZero.clicked.connect(self.send_set_zero_command)
        self.ui.btnClearOutput.clicked.connect(self.clear_output_text)
        self.ui.checkAutoUpdate.stateChanged.connect(self.update_auto_platform)

    def init_com_ports(self):
        arduino_port = self.find_arduino_port()
        if arduino_port:
            self.ui.COMComboBox.setCurrentText(arduino_port)
        ports = [port.device for port in serial.tools.list_ports.comports()]
        current_ports = [self.ui.COMComboBox.itemText(i) for i in range(self.ui.COMComboBox.count())]
        
        if ports != current_ports:
            self.ui.COMComboBox.clear()
            # Ensure arduino_port is at the top and not duplicated
            if arduino_port:
                ports = [arduino_port] + [p for p in ports if p != arduino_port]
            # Add "(Arduino)" next to the Arduino port
            for port in ports:
                if port == arduino_port:
                    self.ui.COMComboBox.addItem(f"{port} (Arduino)")
                else:
                    self.ui.COMComboBox.addItem(port)
            
    def init_baudrate_options(self):
        baudrates = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
        for rate in baudrates:
            self.ui.BaudrateComboBox.addItem(str(rate))
        self.ui.BaudrateComboBox.setCurrentText("115200")
    def find_arduino_port(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            description = port.description.lower()
            if ("arduino" in description or
                "ch340" in description or
                "wchusbserial" in port.device.lower() or
                "usb serial" in description):
                return port.device  # e.g., "COM3" or "/dev/ttyUSB0"
        return None
    
    def toggle_connection(self):
        if self.reader is None:
            port = self.ui.COMComboBox.currentText().split()[0]
            baud = int(self.ui.BaudrateComboBox.currentText())
            self.reader = SerialReader(port, baud, callback=self.update_values)
            output = self.reader.start()
            self.ui.btnConnect.setText("Disconnect")
            self.append_output(output)
        else:
            output = self.reader.stop()
            self.reader = None
            self.ui.btnConnect.setText("Connect")
            self.append_output(output)
    def update_values(self, data):
        filtered_data = {}
        lowpass_filtered_data = {}
        for key in ['lc1', 'lc2', 'lc3', 'lc4', 'lc5', 'lc6']:
            if key in data:
                self.raw_data[key].append(data[key])
                filtered_data[key] = self.kalman_filters[key].update(data[key])
                lowpass_filtered_data[key] = self.lowpass_filters[key].update(data[key])
        if self.ui.radioKF.isChecked():
            # Apply Kalman filter to loadcell values
            self.ui.labelLC1.setText(f"{filtered_data['lc1']:.2f}")
            self.ui.labelLC2.setText(f"{filtered_data['lc2']:.2f}")
            self.ui.labelLC3.setText(f"{filtered_data['lc3']:.2f}")
            self.ui.labelLC4.setText(f"{filtered_data['lc4']:.2f}")
            self.ui.labelLC5.setText(f"{filtered_data['lc5']:.2f}")
            self.ui.labelLC6.setText(f"{filtered_data['lc6']:.2f}")
        elif self.ui.radioLPF.isChecked():
            # Apply Extended Kalman filter to loadcell values
            self.ui.labelLC1.setText(f"{lowpass_filtered_data['lc1']:.2f}")
            self.ui.labelLC2.setText(f"{lowpass_filtered_data['lc2']:.2f}")
            self.ui.labelLC3.setText(f"{lowpass_filtered_data['lc3']:.2f}")
            self.ui.labelLC4.setText(f"{lowpass_filtered_data['lc4']:.2f}")
            self.ui.labelLC5.setText(f"{lowpass_filtered_data['lc5']:.2f}")
            self.ui.labelLC6.setText(f"{lowpass_filtered_data['lc6']:.2f}")
        else:
            self.ui.labelLC1.setText(f"{data['lc1']:.2f}")
            self.ui.labelLC2.setText(f"{data['lc2']:.2f}")
            self.ui.labelLC3.setText(f"{data['lc3']:.2f}")
            self.ui.labelLC4.setText(f"{data['lc4']:.2f}")
            self.ui.labelLC5.setText(f"{data['lc5']:.2f}")
            self.ui.labelLC6.setText(f"{data['lc6']:.2f}")
        loadcells = [data[f'lc{i+1}'] for i in range(6)]
        filtered_loadcells = [filtered_data[key] for key in ['lc1', 'lc2', 'lc3', 'lc4', 'lc5', 'lc6']]
        filtered_2_loadcells = [lowpass_filtered_data[key] for key in ['lc1', 'lc2', 'lc3', 'lc4', 'lc5', 'lc6']]
        # B, P = basePoints, platformPoints (3x6 numpy array) — define globally or load from GUI config
        forces = computeForceMoment(loadcells, self.BasePoint, self.PlatformPoint)
        filtered_forces = computeForceMoment(filtered_loadcells, self.BasePoint, self.PlatformPoint)
        filtered_2_forces = computeForceMoment(filtered_2_loadcells, self.BasePoint, self.PlatformPoint)
        # Apply Kalman filter to force and moment values for smoothing
        fx_kalman = filtered_forces['Fx']
        fy_kalman = filtered_forces['Fy']
        fz_kalman = filtered_forces['Fz']
        mx_kalman = filtered_forces['Mx']
        my_kalman = filtered_forces['My']
        mz_kalman = filtered_forces['Mz']
        # Apply Extended Kalman filter to force and moment values for smoothing
        for key, value in zip(['fx', 'fy', 'fz', 'mx', 'my', 'mz'],
                              [fx_kalman, fy_kalman, fz_kalman, mx_kalman, my_kalman, mz_kalman]):
            self.KF_data[key].append(value)
        for key, value in zip(['fx', 'fy', 'fz', 'mx', 'my', 'mz'],
                              [filtered_2_forces['Fx'], filtered_2_forces['Fy'], filtered_2_forces['Fz'],
                               filtered_2_forces['Mx'], filtered_2_forces['My'], filtered_2_forces['Mz']]):
            self.filtered_2_data[key].append(value)
        if self.ui.radioKF.isChecked():
            self.ui.labelFx.setText(f"{fx_kalman:.2f}")
            self.ui.labelFy.setText(f"{fy_kalman:.2f}")
            self.ui.labelFz.setText(f"{fz_kalman:.2f}")
            self.ui.labelMx.setText(f"{mx_kalman:.2f}")
            self.ui.labelMy.setText(f"{my_kalman:.2f}")
            self.ui.labelMz.setText(f"{mz_kalman:.2f}")
        elif self.ui.radioLPF.isChecked():
            self.ui.labelFx.setText(f"{filtered_2_forces['Fx']:.2f}")
            self.ui.labelFy.setText(f"{filtered_2_forces['Fy']:.2f}")
            self.ui.labelFz.setText(f"{filtered_2_forces['Fz']:.2f}")
            self.ui.labelMx.setText(f"{filtered_2_forces['Mx']:.2f}")
            self.ui.labelMy.setText(f"{filtered_2_forces['My']:.2f}")
            self.ui.labelMz.setText(f"{filtered_2_forces['Mz']:.2f}")
        else:
            self.ui.labelFx.setText(f"{forces['Fx']:.2f}")
            self.ui.labelFy.setText(f"{forces['Fy']:.2f}")
            self.ui.labelFz.setText(f"{forces['Fz']:.2f}")
            self.ui.labelMx.setText(f"{forces['Mx']:.2f}")
            self.ui.labelMy.setText(f"{forces['My']:.2f}")
            self.ui.labelMz.setText(f"{forces['Mz']:.2f}")

        if not self.time_data:
            self.start_time = time.time()
        current_time = time.time()
        now = current_time - self.start_time
        self.time_data.append(now)
        # Extract lists for plotting
        if self.logging_enabled:
            self.raw_data_log.append((now, data['lc1'], data['lc2'], data['lc3'], data['lc4'], data['lc5'], data['lc6']))
            self.force_log.append((now, forces['Fx'], forces['Fy'], forces['Fz']))     # tuple: (t, Fx, Fy, Fz)
            self.moment_log.append((now, forces['Mx'], forces['My'], forces['Mz']))    # tuple: (t, Mx, My, Mz)
            self.kalman_force_log.append((now, fx_kalman, fy_kalman, fz_kalman))
            self.kalman_moment_log.append((now, mx_kalman, my_kalman, mz_kalman))
            self.filtered_2_force_log.append((now, filtered_2_forces['Fx'], filtered_2_forces['Fy'], filtered_2_forces['Fz']))
            self.filtered_2_moment_log.append((now, filtered_2_forces['Mx'], filtered_2_forces['My'], filtered_2_forces['Mz']))
            
            MAX_POINTS = 1000  # or 2000 if needed
            ## Maintain limited history for smoother plotting
            self.raw_data_log = self.raw_data_log[-MAX_POINTS:]
            self.force_log = self.force_log[-MAX_POINTS:]
            self.kalman_force_log = self.kalman_force_log[-MAX_POINTS:]
            self.moment_log = self.moment_log[-MAX_POINTS:]
            self.kalman_moment_log = self.kalman_moment_log[-MAX_POINTS:]
            self.filtered_2_force_log = self.filtered_2_force_log[-MAX_POINTS:]
            self.filtered_2_moment_log = self.filtered_2_moment_log[-MAX_POINTS:]
            # Update plot data
            
            times, lc1, lc2, lc3, lc4, lc5, lc6 = zip(*self.raw_data_log)
            times, fx, fy, fz = zip(*self.force_log)
            times, mx, my, mz = zip(*self.moment_log)
            times, kfx, kfy, kfz = zip(*self.kalman_force_log)
            times, kmx, kmy, kmz = zip(*self.kalman_moment_log)
            times, filtered_2_forces['Fx'], filtered_2_forces['Fy'], filtered_2_forces['Fz'] = zip(*self.filtered_2_force_log)
            times, filtered_2_forces['Mx'], filtered_2_forces['My'], filtered_2_forces['Mz'] = zip(*self.filtered_2_moment_log)
            # Update plot
            # self.update_force_plot()
            # self.update_moment_plot()
            # Log current data
            self.log_current_data(times[-1], lc1[-1], lc2[-1], lc3[-1], lc4[-1], lc5[-1], lc6[-1],
                                  fx[-1], fy[-1], fz[-1], mx[-1], my[-1], mz[-1],
                                    kfx[-1], kfy[-1], kfz[-1], kmx[-1], kmy[-1], kmz[-1],
                                    filtered_2_forces['Fx'][-1], filtered_2_forces['Fy'][-1], filtered_2_forces['Fz'][-1],
                                    filtered_2_forces['Mx'][-1], filtered_2_forces['My'][-1], filtered_2_forces['Mz'][-1])
    def save_config(self):
        config = {
            "Rb": self.ui.RadiusBaseSpinBox.value(),
            "Rp": self.ui.RadiusPlatformSpinBox.value(),
            "legLength": self.ui.LegLengthSpinBox.value(),  # 👈 use leg length
            "thetaB": [
                self.ui.ThetaB1SpinBox.value(), self.ui.ThetaB2SpinBox.value(),
                self.ui.ThetaB3SpinBox.value(), self.ui.ThetaB4SpinBox.value(),
                self.ui.ThetaB5SpinBox.value(), self.ui.ThetaB6SpinBox.value()
            ],
            "thetaP": [
                self.ui.ThetaP1SpinBox.value(), self.ui.ThetaP2SpinBox.value(),
                self.ui.ThetaP3SpinBox.value(), self.ui.ThetaP4SpinBox.value(),
                self.ui.ThetaP5SpinBox.value(), self.ui.ThetaP6SpinBox.value()
            ]
        }

        file, _ = QFileDialog.getSaveFileName(self, "Save Config", "", "JSON Files (*.json)")
        if file:
            with open(file, 'w') as f:
                json.dump(config, f, indent=4)
        self.append_output(f"Config saved to {file}")
    def load_config(self):
        file, _ = QFileDialog.getOpenFileName(self, "Load Config", "", "JSON Files (*.json)")
        if file:
            with open(file, 'r') as f:
                config = json.load(f)

            self.ui.RadiusBaseSpinBox.setValue(config["Rb"])
            self.ui.RadiusPlatformSpinBox.setValue(config["Rp"])
            self.ui.LegLengthSpinBox.setValue(config["legLength"])  # 👈 updated here

            # Set GUI values from config
            for i in range(6):
                getattr(self.ui, f"ThetaB{i+1}SpinBox").setValue(config["thetaB"][i])
                getattr(self.ui, f"ThetaP{i+1}SpinBox").setValue(config["thetaP"][i])

            # Recompute geometry
            B, P = build_geometry_from_config(
                config["Rb"], config["Rp"], config["thetaB"], config["thetaP"], config["legLength"]
            )
            self.BasePoint = B
            self.PlatformPoint = P
        self.append_output(f"[File] Config loaded from {file}")
    
    def apply_config(self):

        # Extract parameters from UI
        Rb = self.ui.RadiusBaseSpinBox.value()
        Rp = self.ui.RadiusPlatformSpinBox.value()
        legLength = self.ui.LegLengthSpinBox.value()

        thetaB = [getattr(self.ui, f"ThetaB{i+1}SpinBox").value() for i in range(6)]
        thetaP = [getattr(self.ui, f"ThetaP{i+1}SpinBox").value() for i in range(6)]

        # Recalculate geometry
        try:
            self.BasePoint, self.PlatformPoint = build_geometry_from_config(
            Rb, Rp, thetaB, thetaP, legLength
            )
            self.plot_stewart()  # Optional: if you want immediate plotting
        except Exception as e:
            QMessageBox.critical(self, "Config Error", f"Failed to apply config:\n{str(e)}")
    def plot_3D_setup(self):
        # 1. Create the Figure and Canvas only once
        self.plot_fig = Figure(figsize=(1, 1))
        self.plot_ax = self.plot_fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvas(self.plot_fig)
        self.plot_ax.view_init(elev=30, azim=60)

        # 2. Set the layout just once
        self.plot_layout = QVBoxLayout()
        self.plot_layout.addWidget(self.canvas)
        self.ui.graphWidget.setLayout(self.plot_layout)

        ## Figure and Canvas setup

    def plot_stewart(self):
        fig = self.plot_fig 
        ax = self.plot_ax
        ax.clear()  # Clear previous plot
        # Set labels and title
        ax.set_title("Stewart Platform Visualization")
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")

        B = self.BasePoint  # 3x6
        P = self.PlatformPoint  # 3x6

        # Try to get loadcell values from the UI, fallback to zeros if not available
        try:
            loadcells = [
                float(self.ui.labelLC1.text()),
                float(self.ui.labelLC2.text()),
                float(self.ui.labelLC3.text()),
                float(self.ui.labelLC4.text()),
                float(self.ui.labelLC5.text()),
                float(self.ui.labelLC6.text())
            ]
        except Exception:
            loadcells = [0] * 6
        norm = plt.Normalize(vmin=-5, vmax=5)
        cmap = plt.get_cmap('jet')  # 'jet' goes from blue → green → red
        colors = [cmap(norm(lc)) for lc in loadcells]
        # Plot base and platform points
        ax.scatter(B[0], B[1], B[2], c='blue', s=50, label='Base Joints')
        ax.scatter(P[0], P[1], P[2], c='red', s=50, label='Platform Joints')

        # Connect legs and add name tags with color gradient
        for i in range(6):
            ax.plot([B[0, i], P[0, i]],
                    [B[1, i], P[1, i]],
                    [B[2, i], P[2, i]],
                    color=colors[i], linewidth=4)
            # Add name tag at the midpoint of each leg
            mid_x = (B[0, i] + P[0, i]) / 2
            mid_y = (B[1, i] + P[1, i]) / 2
            mid_z = (B[2, i] + P[2, i]) / 2
            ax.text(mid_x, mid_y, mid_z, f"Leg {i+1}", color='black', fontsize=9, ha='center', va='center')

        # Optionally draw outlines
        ax.plot_trisurf(B[0], B[1], B[2], color='blue', alpha=0.1)
        ax.plot_trisurf(P[0], P[1], P[2], color='red', alpha=0.1)

        ax.legend(loc = 'upper left')  # Add legend to the plot

        # Add colorbar (gradient scale) beside the plot only once
        if not hasattr(self, 'colorbar') or self.colorbar is None:
            sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
            sm.set_array([])
            self.colorbar = fig.colorbar(sm, ax=ax, shrink=0.5, aspect=10, label="Loadcell Value")

        self.canvas.draw()  # Redraw the updated plot
    def plot_2D_setup(self):
        self.force_lines = {} # For raw force lines
        self.kalman_lines = {} # For Kalman Filter lines
        self.f2_lines = {} # For Extended Kalman Filter lines
        for key in ['fx', 'fy', 'fz', 'mx', 'my', 'mz']:
            fig = Figure(figsize=(3, 1.5))
            ax = fig.add_subplot(111)
            canvas = FigureCanvas(fig)

            self.force_figures[key] = fig
            self.force_axes[key] = ax
            self.force_canvases[key] = canvas

            raw_line, = ax.plot([], [], '.-', color='gray', label='Raw')
            kalman_line, = ax.plot([], [], color='red', label='KF')
            f2_line, = ax.plot([], [], color='blue', label='LPF')
            
            ax.set_title(key.capitalize() + " vs Time")
            ax.set_xlabel("Time")
            ax.set_ylabel("Force (N)" if 'f' in key else "Moment (Nmm)")
            ax.grid(True)
            ax.legend(loc='upper left')  # or try 'lower left', etc.

            self.force_lines[key] = raw_line
            self.kalman_lines[key] = kalman_line
            self.f2_lines[key] = f2_line

            plot_area = getattr(self.ui, f"{'force' if 'f' in key else 'moment'}_{key[-1]}_PlotArea", None)
            if plot_area and plot_area.layout():
                plot_area.layout().addWidget(canvas)
                
    def update_force_moment_plots_frequent(self):
        if not self.plotting_enabled:
            return
        if not self.force_log or not self.moment_log:
            return
        self.update_force_plot()
        self.update_moment_plot()
    def update_force_plot(self):
        if not self.force_log or not self.kalman_force_log:
            return
        times = [point[0] for point in self.force_log]
        Fx = [point[1] for point in self.force_log]
        Fy = [point[2] for point in self.force_log]
        Fz = [point[3] for point in self.force_log]
        kFx = [point[1] for point in self.kalman_force_log]
        kFy = [point[2] for point in self.kalman_force_log]
        kFz = [point[3] for point in self.kalman_force_log]
        f2_Fx = [point[1] for point in self.filtered_2_force_log]
        f2_Fy = [point[2] for point in self.filtered_2_force_log]
        f2_Fz = [point[3] for point in self.filtered_2_force_log]
        N = self.plot_limit
        for key, raw, KF_val, f2_val in zip(['fx', 'fy', 'fz'], [Fx, Fy, Fz], [kFx, kFy, kFz], [f2_Fx, f2_Fy, f2_Fz]):
            if not self.ui.checkFx.isChecked() and key == 'fx':
                continue
            if not self.ui.checkFy.isChecked() and key == 'fy':
                continue
            if not self.ui.checkFz.isChecked() and key == 'fz':
                continue
            self.force_lines[key].set_data(times[-N:], raw[-N:])
            if self.ui.checkKF.isChecked():
                self.kalman_lines[key].set_data(times[-N:], KF_val[-N:])
            else:
                self.kalman_lines[key].set_data([], [])
            if self.ui.checkLPF.isChecked():
                self.f2_lines[key].set_data(times[-N:], f2_val[-N:])
            else:
                self.f2_lines[key].set_data([], [])
            ax = self.force_axes[key]
            ax.relim()
            ax.autoscale_view()
            self.force_canvases[key].draw()
    def update_moment_plot(self):
        if not self.moment_log:
            return
        times = [point[0] for point in self.moment_log]
        Mx = [point[1] for point in self.moment_log]
        My = [point[2] for point in self.moment_log]
        Mz = [point[3] for point in self.moment_log]
        kMx = [point[1] for point in self.kalman_moment_log]
        kMy = [point[2] for point in self.kalman_moment_log]
        kMz = [point[3] for point in self.kalman_moment_log]
        f2_Mx = [point[1] for point in self.filtered_2_moment_log]
        f2_My = [point[2] for point in self.filtered_2_moment_log]
        f2_Mz = [point[3] for point in self.filtered_2_moment_log]
        N = self.plot_limit
        for key, raw, KF_val, f2_val in zip(['mx', 'my', 'mz'], [Mx, My, Mz],[kMx, kMy, kMz], [f2_Mx, f2_My,f2_Mz]):
            if not self.ui.checkMx.isChecked() and key == 'mx':
                continue
            if not self.ui.checkMy.isChecked() and key == 'my':
                continue
            if not self.ui.checkMz.isChecked() and key == 'mz':
                continue
            self.force_lines[key].set_data(times[-N:], raw[-N:])
            if self.ui.checkKF.isChecked():
                self.kalman_lines[key].set_data(times[-N:], KF_val[-N:])
            else:
                self.kalman_lines[key].set_data([], [])
            if self.ui.checkLPF.isChecked():
                self.f2_lines[key].set_data(times[-N:], f2_val[-N:])
            else:
                self.f2_lines[key].set_data([], [])
            ax = self.force_axes[key]
            ax.relim()
            ax.autoscale_view()
            self.force_canvases[key].draw()
    def send_set_zero_command(self):
        if self.reader and self.reader.serial and self.reader.serial.is_open:
            try:
                self.reader.serial.write(b't\n')  # Replace with your actual MCU command
                self.append_output("[Command] Sent 'Set Zero' to MCU")
            except Exception as e:
                self.append_output(f"[Error] Failed to send 'Set Zero': {e}")
        else:
            self.append_output("[Error] Serial port is not open.")
    
    def append_output(self, message):
        self.ui.outputTextEdit.append(message)  # replace with actual QTextEdit object name
    def clear_output_text(self):
        self.ui.outputTextEdit.clear()
    def update_auto_platform(self):
        if self.ui.checkAutoUpdate.isChecked():
            self.plot_timer.start(2000)  # Stop manual plotting
        else: 
            self.plot_timer.stop()  # Stop manual plotting     
            
    def start_logging(self):
        # Check if serial port is open
            # If not, show warning and return
        if self.reader is None or not self.reader.serial or not self.reader.serial.is_open:
            self.append_output("[Serial Port] ⚠️ Serial port is not open. Cannot start logging.")
            return
        # If logging is already enabled, show warning and return
        if self.logging_enabled:
            self.append_output("[Log] ⚠️ Logging is already enabled.")
            return
        self.ui.Plotter.setCurrentIndex(self.ui.Plotter.indexOf(self.ui.plotTab))
        # Start logging
        self.logging_enabled = True
        self.plotting_enabled = True
        self.append_output("[Log] 🔴 Logging started")
        self.time_data.clear()
        self.force_log.clear()
        self.moment_log.clear()
        self.log_data.clear()
        self.kalman_force_log.clear()
        self.kalman_moment_log.clear()
        self.filtered_2_force_log.clear()
        self.filtered_2_moment_log.clear()
    def stop_logging(self):
        if self.logging_enabled:
            self.logging_enabled = False
            self.plotting_enabled = False  # 👈 Stop plots too
            self.append_output("[Log] 🟢 Logging stopped")
        else:
            self.append_output("[Log] ⚠️ Logging is already stop.")
    def log_current_data(self, times, lc1, lc2, lc3, lc4, lc5, lc6, fx, fy, fz, mx, my, mz, kfx, kfy, kfz, kmx, kmy, kmz, f2_fx, f2_fy, f2_fz, f2_mx, f2_my, f2_mz):
        if self.logging_enabled:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")
            self.log_data.append((times, lc1, lc2, lc3, lc4, lc5, lc6, fx, fy, fz, mx, my, mz, kfx, kfy, kfz, kmx, kmy, kmz, f2_fx, f2_fy, f2_fz, f2_mx, f2_my, f2_mz))

    def export_csv(self):
        if not self.log_data:
            self.append_output("[File] ⚠️ No data to export.")
            return

        file_path, _ = QFileDialog.getSaveFileName(self, "Save CSV", "", "CSV Files (*.csv)")
        if not file_path:
            return

        try:
            with open(file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Timestamp (s)","L1 (N)","L2 (N)","L3 (N)","L4 (N)","L5 (N)","L6 (N)", "Fx (N)", "Fy (N)", "Fz (N)", "Mx (Nmm)", "My (Nmm)", "Mz (Nmm)",
                                 "Fx_kalman (N)", "Fy_kalman (N)", "Fz_kalman (N)", "Mx_kalman (Nmm)", "My_kalman (Nmm)", "Mz_kalman (Nmm)",
                                 "Fx_Lowpass (N)", "Fy_Lowpass (N)", "Fz_Lowpass (N)", "Mx_Lowpass (Nmm)", "My_Lowpass (Nmm)", "Mz_Lowpass (Nmm)"])
                writer.writerows(self.log_data)
            self.append_output(f"[File] ✅ CSV exported: {file_path}")
        except PermissionError:
            self.append_output(f"[File] ❌ Permission denied: {file_path}. Please close the file if it is open in another program and try again.")
        except Exception as e:
            self.append_output(f"[File] ❌ Failed to export CSV: {e}")
        
    def closeEvent(self, event):
        if self.reader:
            self.reader.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = StewartApp()
    win.show()
    sys.exit(app.exec_())
