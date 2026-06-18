import csv
import random

# Constants
x = 0.5  # Example value, adjust as needed
z = 0.3  # Example value, adjust as needed

# Example force values
forces = [
    {"F_x": 10, "F_z": 20},
    {"F_x": 15, "F_z": 25},
    {"F_x": 20, "F_z": 30},
    {"F_x": 25, "F_z": 35},
    {"F_x": 30, "F_z": 40},
    {"F_x": 35, "F_z": 45},
    {"F_x": 40, "F_z": 50},
    {"F_x": 45, "F_z": 55},
    {"F_x": 50, "F_z": 60},
    {"F_x": 55, "F_z": 65},
    {"F_x": 60, "F_z": 70},
    {"F_x": 65, "F_z": 75},
    {"F_x": 70, "F_z": 80},
    {"F_x": 75, "F_z": 85},
    {"F_x": 80, "F_z": 90},
    {"F_x": 85, "F_z": 95},
    {"F_x": 90, "F_z": 100},
    {"F_x": 95, "F_z": 105},
    {"F_x": 100, "F_z": 110},
    {"F_x": 105, "F_z": 115},
    {"F_x": 110, "F_z": 120},
    {"F_x": 115, "F_z": 125},
    {"F_x": 120, "F_z": 130},
    {"F_x": 125, "F_z": 135},
    {"F_x": 130, "F_z": 140},
]

# Calculate M_pitch and write to CSV
with open('M_pitch.csv', 'w', newline='') as csvfile:
    fieldnames = ['Fx', 'Fz', 'Mpitch']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    
    writer.writeheader()
    for force in forces:
        M_pitch = force['F_x'] * x + force['F_z'] * z
        noise = random.gauss(0, 0.5)  # Gaussian noise with mean=0, std=0.5
        M_pitch_noisy = M_pitch + noise
        writer.writerow({
            'Fx': force['F_x'],
            'Fz': force['F_z'],
            'Mpitch': M_pitch_noisy
        })

print("CSV file 'M_pitch.csv' generated successfully!")
