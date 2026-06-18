import pandas as pd
import numpy as np

class KalmanFilter1D:
    def __init__(self, process_variance, measurement_variance, initial_value=0, initial_estimate_error=1):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = initial_value
        self.estimate_error = initial_estimate_error
        
    def update(self, measurement):
        # Predict
        prediction = self.estimate
        prediction_error = self.estimate_error + self.process_variance
        
        # Update
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = prediction + kalman_gain * (measurement - prediction)
        self.estimate_error = (1 - kalman_gain) * prediction_error
        
        return self.estimate

def apply_kalman_filter(input_csv, output_csv, process_variance=0.0001, measurement_variance=0.05):
    # Read CSV file
    df = pd.read_csv(input_csv)
    
    # Initialize Kalman filters for each column
    kf_fx = KalmanFilter1D(process_variance, measurement_variance)
    kf_fz = KalmanFilter1D(process_variance, measurement_variance)
    kf_my = KalmanFilter1D(process_variance, measurement_variance)
    
    # Apply Kalman filter
    df['Fx_kalman (N)'] = df['Fx (N)'].apply(kf_fx.update)
    df['Fz_kalman (N)'] = df['Fz (N)'].apply(kf_fz.update)
    df['My_kalman (Nmm)'] = df['My (Nmm)'].apply(kf_my.update)
    
    # Save to new CSV file
    df.to_csv(output_csv, index=False)
    print(f"Filtered data saved to {output_csv}")

if __name__ == "__main__":
    input_file = "Excel/Mx_measurement.csv"  # Change to your input file
    output_file = "Mx_measurement_kalman.csv"
    
    apply_kalman_filter(input_file, output_file)