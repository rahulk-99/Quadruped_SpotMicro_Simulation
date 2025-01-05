import numpy as np
import sympy
from sympy import Matrix, pi, cos, sin
import matplotlib.pyplot as plt

# Define symbolic variables
theta1, theta2, theta3, l1, l2, l3 = sympy.symbols("theta1, theta2, theta3, l1, l2, l3")

# Transformation matrix from DH parameters
def transformation_matrix_from_dh_param(dh_row):
    theta, alpha, a, d = dh_row
    return Matrix([
        [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0, sin(alpha), cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Define DH tables for both legs
left_leg_dh_table = Matrix([
    [theta1, -pi/2, 0, 0],
    [0, 0, 0, l1],
    [theta2 + pi/4, 0, l2, 0],
    [theta3 - pi/2, 0, l3, 0]
])

right_leg_dh_table = Matrix([
    [theta1, pi/2, 0, 0],
    [0, 0, 0, l1],
    [theta2 - pi/4, 0, l2, 0],
    [theta3 + pi/2, 0, l3, 0]
])

# Calculate transformation matrices
def calculate_transformation_matrix(dh_table):
    return [transformation_matrix_from_dh_param(dh_table[i, :]) for i in range(dh_table.rows)]

def forward_kinematics(transformation_matrix):
    matrix = Matrix.eye(4)
    fk_steps = []  # Store intermediate transformations
    for transform in transformation_matrix:
        matrix = matrix * transform
        fk_steps.append(matrix)
    return matrix, fk_steps

# Generate FK for each leg
fk_left, fk_left_steps = forward_kinematics(calculate_transformation_matrix(left_leg_dh_table))
fk_right, fk_right_steps = forward_kinematics(calculate_transformation_matrix(right_leg_dh_table))

# Define joint configurations and validate FK
configurations = [
    {"leg": "Left", "theta1": 0, "theta2": -pi/4, "theta3": pi/2, "l1": 0.0255, "l2": 0.10921, "l3": 0.13528},
    {"leg": "Left", "theta1": 0, "theta2": -pi/6, "theta3": pi/3, "l1": 0.0255, "l2": 0.10921, "l3": 0.13528},
    {"leg": "Right", "theta1": 0, "theta2": pi/4, "theta3": -pi/2, "l1": 0.0255, "l2": 0.10921, "l3": 0.13528},
    {"leg": "Right", "theta1": 0, "theta2": pi/6, "theta3": -pi/3, "l1": 0.0255, "l2": 0.10921, "l3": 0.13528},
]

for config in configurations:
    leg = config["leg"]
    theta1_val, theta2_val, theta3_val = config["theta1"], config["theta2"], config["theta3"]
    l1_val, l2_val, l3_val = config["l1"], config["l2"], config["l3"]
   
    fk_matrix = fk_left if leg == "Left" else fk_right
    fk_result = fk_matrix.subs({
        theta1: theta1_val,
        theta2: theta2_val,
        theta3: theta3_val,
        l1: l1_val,
        l2: l2_val,
        l3: l3_val
    }).evalf()
   
    print(f"{leg} Leg Configuration: theta1={theta1_val}, theta2={theta2_val}, theta3={theta3_val}")
    print("Transformation Matrix:")
    sympy.pprint(fk_result)
    print("-" * 50)

import numpy as np
import sympy
from sympy import Matrix, cos, sin, pi
import matplotlib.pyplot as plt

# DH Parameters
def transformation_matrix_from_dh_param(dh_row):
    theta, alpha, a, d = dh_row
    return Matrix([
        [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0,           sin(alpha),             cos(alpha),            d],
        [0,           0,                      0,                     1]
    ])

def calculate_transformation_matrix(dh_table):
    return [transformation_matrix_from_dh_param(dh_table[i:i+4]) for i in range(0, dh_table.rows * 4, 4)]

def forward_kinematics(dh_table):
    matrices = calculate_transformation_matrix(dh_table)
    T = Matrix.eye(4)  # Identity matrix
    joint_positions = [T[:3, 3]]  # Start at the base origin
    for transform in matrices:
        T = T * transform
        joint_positions.append(T[:3, 3])  # Extract position
    return joint_positions

# Left Leg DH Table
left_leg_dh_table = Matrix([
    [0,        -pi/2, 0,     0],
    [0,         0,    0,  0.0255],
    [-pi/3+pi/4, 0, 0.10921, 0],
    [pi/6-pi/2,  0, 0.13528, 0]
])

# Right Leg DH Table
right_leg_dh_table = Matrix([
    [0,         pi/2, 0,     0],
    [0,         0,    0,  0.0255],
    [pi/3-pi/4,  0, 0.10921, 0],
    [-pi/6+pi/2, 0, 0.13528, 0]
])

# Visualize Forward Kinematics
def plot_kinematics(joint_positions, title, ax):
    x, y, z = zip(*[(float(pos[0]), float(pos[1]), float(pos[2])) for pos in joint_positions])
    ax.plot(x, y, z, marker='o', label=title)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend()
    ax.set_title(title)

fig = plt.figure(figsize=(12, 6))

# Left Leg
ax1 = fig.add_subplot(121, projection='3d')
left_leg_positions = forward_kinematics(left_leg_dh_table)
plot_kinematics(left_leg_positions, "Left Leg Forward Kinematics", ax1)

# Right Leg
ax2 = fig.add_subplot(122, projection='3d')
right_leg_positions = forward_kinematics(right_leg_dh_table)
plot_kinematics(right_leg_positions, "Right Leg Forward Kinematics", ax2)

plt.tight_layout()
plt.show()
