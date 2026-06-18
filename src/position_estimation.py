import sys
import numpy as np
import pandas as pd

from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QFileDialog,
    QVBoxLayout, QLabel
)

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.patches import Ellipse


class ForcePositionEstimator(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Force Position Estimation (Least Squares)")

        self.data = None

        self.btn_import = QPushButton("Import CSV")
        self.btn_calc = QPushButton("Calculate x, z")
        self.lbl_result = QLabel("x = ?, z = ?")
        self.lbl_acc = QLabel("σx = ?, σz = ?")

        self.fig = Figure(figsize=(6, 4))
        self.canvas = FigureCanvasQTAgg(self.fig)

        layout = QVBoxLayout()
        layout.addWidget(self.btn_import)
        layout.addWidget(self.btn_calc)
        layout.addWidget(self.lbl_result)
        layout.addWidget(self.lbl_acc)
        layout.addWidget(self.canvas)

        self.setLayout(layout)

        self.btn_import.clicked.connect(self.load_csv)
        self.btn_calc.clicked.connect(self.calculate)

    def load_csv(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open CSV", "", "CSV Files (*.csv)"
        )
        if path:
            self.data = pd.read_csv(path)
            self.lbl_result.setText("CSV loaded. Ready.")

    def calculate(self):
        if self.data is None:
            return

        Fx = self.data["Fx"].values
        Fz = self.data["Fz"].values
        M  = self.data["Mpitch"].values

        A = np.column_stack((Fx, Fz))

        # Least squares
        p_hat, residuals, rank, s = np.linalg.lstsq(A, M, rcond=None)
        x_hat, z_hat = p_hat

        # Reconstructed moment
        M_hat = A @ p_hat

        # Noise variance estimate
        N = len(M)
        sigma2 = np.sum((M - M_hat)**2) / (N - 2)

        # Covariance of estimates
        Cov = sigma2 * np.linalg.inv(A.T @ A)
        sigma_x = np.sqrt(Cov[0, 0])
        sigma_z = np.sqrt(Cov[1, 1])

        self.lbl_result.setText(f"x = {x_hat:.4f}, z = {z_hat:.4f}")
        self.lbl_acc.setText(f"σx = {sigma_x:.4e}, σz = {sigma_z:.4e}")

        # ===== Plot =====
        self.fig.clear()

        # ---- Plot 1: Moment comparison ----
        ax1 = self.fig.add_subplot(211)
        ax1.plot(M, label="Measured Mpitch")
        ax1.plot(M_hat, "--", label="Estimated Mpitch")
        ax1.set_ylabel("Moment")
        ax1.legend()
        ax1.grid(True)

        # ---- Plot 2: Force position geometry + confidence ellipse ----
        ax2 = self.fig.add_subplot(212)

        # Origin
        ax2.plot(0, 0, "ko", label="Origin")

        # Estimated force application point
        ax2.plot(x_hat, z_hat, "ro", label="Estimated force point")

        # Mean force direction (visual cue)
        Fx_mean = np.mean(Fx)
        Fz_mean = np.mean(Fz)

        ax2.arrow(
            x_hat, z_hat,
            Fx_mean * 0.1, Fz_mean * 0.1,
            head_width=0.02,
            length_includes_head=True,
            color="r"
        )

        # ---- Confidence ellipse ----
        eigvals, eigvecs = np.linalg.eig(Cov)

        # Sort eigenvalues (largest first)
        order = eigvals.argsort()[::-1]
        eigvals = eigvals[order]
        eigvecs = eigvecs[:, order]

        # 95% confidence scaling
        chi2_val = 5.991

        width  = 2 * np.sqrt(chi2_val * eigvals[0])
        height = 2 * np.sqrt(chi2_val * eigvals[1])

        angle = np.degrees(np.arctan2(eigvecs[1, 0], eigvecs[0, 0]))

        ellipse = Ellipse(
            (x_hat, z_hat),
            width=width,
            height=height,
            angle=angle,
            edgecolor="r",
            facecolor="none",
            linestyle="--",
            linewidth=2,
            label="95% confidence region"
        )

        ax2.add_patch(ellipse)

        ax2.set_xlabel("x")
        ax2.set_ylabel("z")
        ax2.set_aspect("equal", adjustable="box")
        ax2.grid(True)
        ax2.legend()

        # Origin
        ax2.plot(0, 0, "ko", label="Origin")

        # Estimated force application point
        ax2.plot(x_hat, z_hat, "ro", label="Force application point")

        # Mean force direction (for visualization only)
        Fx_mean = np.mean(Fx)
        Fz_mean = np.mean(Fz)

        ax2.arrow(
            x_hat, z_hat,
            Fx_mean * 0.1, Fz_mean * 0.1,   # scale for visibility
            head_width=0.02,
            length_includes_head=True,
            color="r"
        )

        ax2.set_xlabel("x")
        ax2.set_ylabel("z")
        ax2.set_aspect("equal", adjustable="box")
        ax2.grid(True)
        ax2.legend()

        self.canvas.draw()



if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = ForcePositionEstimator()
    w.show()
    sys.exit(app.exec_())
