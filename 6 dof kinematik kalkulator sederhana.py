import numpy as np
import tkinter as tk
from tkinter import messagebox
from scipy.optimize import minimize

# Fungsi Transformasi DH dengan Offset Theta
def dh_transform(theta, d, a, alpha, offset=0):
    theta = np.radians(theta + offset)  
    alpha = np.radians(alpha)
    T = np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])
    return T

# Default Parameter DH
dh_params = [
    {'theta': 0, 'd': 170, 'a': 75, 'alpha': -90, 'offset': 0},
    {'theta': -90, 'd': 0, 'a': 330, 'alpha': 0, 'offset': 0},
    {'theta': 0, 'd': 0, 'a': 0, 'alpha': -90, 'offset': 90},
    {'theta': 0, 'd': -240, 'a': 0, 'alpha': -90, 'offset': 0},
    {'theta': 0, 'd': 0, 'a': 0, 'alpha': -90, 'offset': 0},
    {'theta': 180, 'd': -40, 'a': 0, 'alpha': 0, 'offset': 0},
]

# Fungsi untuk menghitung posisi end-effector dan orientasi (YPR)
def calculate_end_effector(joint_angles):
    T_total = np.eye(4)
    for i, param in enumerate(dh_params):
        T = dh_transform(joint_angles[i], param['d'], param['a'], param['alpha'], param.get('offset', 0))
        T_total = np.dot(T_total, T)

    x = T_total[0, 3]
    y = T_total[1, 3]
    z = T_total[2, 3]

    yaw = np.degrees(np.arctan2(T_total[1, 0], T_total[0, 0]))
    pitch = np.degrees(np.arctan2(-T_total[2, 0], np.sqrt(T_total[2, 1]**2 + T_total[2, 2]**2)))
    roll = np.degrees(np.arctan2(T_total[2, 1], T_total[2, 2]))

    return x, y, z, yaw, pitch, roll

# Fungsi untuk kinematika mundur
def inverse_kinematics(target_x, target_y, target_z):
    def objective(joint_angles):
        x, y, z, _, _, _ = calculate_end_effector(joint_angles)
        return (x - target_x)**2 + (y - target_y)**2 + (z - target_z)**2

    initial_guess = [0, -90, 90, 0, 0, 0]
    bounds = [
        (-170, 170),
        (-132, 0),
        (1, 141),
        (-165, 165),
        (-105, 105),
        (-155, 155)
    ]

    result = minimize(objective, initial_guess, bounds=bounds)

    if result.success:
        return result.x
    else:
        messagebox.showerror("Error", "Inverse kinematics solution not found")
        return None

# Fungsi untuk update posisi dan orientasi
def update_position():
    try:
        joint_angles = [float(entry.get()) for entry in entries_joints]
        x, y, z, yaw, pitch, roll = calculate_end_effector(joint_angles)
        label_position.config(text=f"Position: x = {x:.2f}, y = {y:.2f}, z = {z:.2f}")
        label_orientation.config(text=f"Orientation: Yaw = {yaw:.2f}°, Pitch = {pitch:.2f}°, Roll = {roll:.2f}°")
    except Exception as e:
        messagebox.showerror("Error", f"Error in input: {e}")

def update_inverse_kinematics():
    try:
        target_x = float(entry_target_x.get())
        target_y = float(entry_target_y.get())
        target_z = float(entry_target_z.get())
    except ValueError:
        messagebox.showerror("Invalid Input", "Please enter valid numerical values for the target position.")
        return

    joint_angles = inverse_kinematics(target_x, target_y, target_z)
    if joint_angles is not None:
        for i, entry in enumerate(entries_joints):
            entry.delete(0, tk.END)
            entry.insert(0, f"{joint_angles[i]:.2f}")

    update_position()

def save_dh_parameters():
    try:
        for i, entry in enumerate(entries_dh):
            values = [float(x) for x in entry.get().split(',')]
            dh_params[i]['theta'] = values[0]
            dh_params[i]['d'] = values[1]
            dh_params[i]['a'] = values[2]
            dh_params[i]['alpha'] = values[3]
            dh_params[i]['offset'] = values[4]
        messagebox.showinfo("Success", "DH Parameters saved successfully!")
    except Exception as e:
        messagebox.showerror("Error", f"Invalid DH parameters: {e}")

root = tk.Tk()
root.title("Kalkulator Kinematik")

entries_joints = []
entries_dh = []

def setup_gui():
    global entry_target_x, entry_target_y, entry_target_z
    global label_position, label_orientation

    tk.Label(root, text="Joint Angles (Degrees):").grid(row=0, column=0, columnspan=2)
    for i in range(6):
        tk.Label(root, text=f"J{i+1}:").grid(row=i+1, column=0)
        entry = tk.Entry(root)
        entry.grid(row=i+1, column=1)
        entries_joints.append(entry)

    tk.Label(root, text="Target Position:").grid(row=7, column=0, columnspan=2)
    tk.Label(root, text="X:").grid(row=8, column=0)
    entry_target_x = tk.Entry(root)
    entry_target_x.grid(row=8, column=1)
    tk.Label(root, text="Y:").grid(row=9, column=0)
    entry_target_y = tk.Entry(root)
    entry_target_y.grid(row=9, column=1)
    tk.Label(root, text="Z:").grid(row=10, column=0)
    entry_target_z = tk.Entry(root)
    entry_target_z.grid(row=10, column=1)

    tk.Button(root, text="Calculate FK", command=update_position).grid(row=11, column=0)
    tk.Button(root, text="Calculate IK", command=update_inverse_kinematics).grid(row=11, column=1)

    tk.Label(root, text="DH Parameters:").grid(row=12, column=0, columnspan=2)
    for i, param in enumerate(dh_params):
        tk.Label(root, text=f"Joint {i+1}:").grid(row=13 + i, column=0)
        entry = tk.Entry(root)
        entry.insert(0, f"{param['theta']},{param['d']},{param['a']},{param['alpha']},{param['offset']}")
        entry.grid(row=13 + i, column=1)
        entries_dh.append(entry)

    tk.Button(root, text="Save DH Parameters", command=save_dh_parameters).grid(row=19, column=0, columnspan=2)

    label_position = tk.Label(root, text="Position: x = 0.00, y = 0.00, z = 0.00")
    label_position.grid(row=20, column=0, columnspan=2)

    label_orientation = tk.Label(root, text="Orientation: Yaw = 0.00°, Pitch = 0.00°, Roll = 0.00°")
    label_orientation.grid(row=21, column=0, columnspan=2)

    root.mainloop()

setup_gui()