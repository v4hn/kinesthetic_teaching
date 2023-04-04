#!/usr/bin/env python

import pandas as pd
from scipy.interpolate import CubicHermiteSpline
import sys
import os

# Get the input filename from the command line arguments
if len(sys.argv) < 2:
    print("Usage: python process_trajectory.py <input_csv>")
    sys.exit(1)

input_filename = sys.argv[1]

# Load the data from the input csv file
data = pd.read_csv(input_filename)

# Compute the time step between samples
dt = data['time_from_start'].diff() #1.0 / 50.0  # 50 Hz

# Extract the joint names from the column names
joint_names = [col.replace('_pos', '') for col in data.columns if col.endswith('_pos')]

# Compute the approximate velocities and accelerations for each joint
for joint in joint_names:
    pos_col = f'{joint}_pos'
    vel_col = f'{joint}_vel'
    acc_col = f'{joint}_acc'
    data[vel_col] = (data[pos_col].diff() / dt).fillna(0)

    data[acc_col+"_nd"] = (data[vel_col].diff() / dt).fillna(0)
    joint_spline = CubicHermiteSpline(data['time_from_start'], data[pos_col], data[vel_col])
    data[acc_col+"_spline"] = joint_spline.derivative(nu=2)(data['time_from_start'])


# Write the updated data to a new csv file with the suffix "_approx_vel_acc"
output_filename = os.path.splitext(input_filename)[0] + '_approx_vel_acc.csv'
data.to_csv(output_filename, index=False)

print(f"Processed trajectory saved to {output_filename}")
