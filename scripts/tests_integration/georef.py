import math

import numpy as np
# Camera model
from scipy.spatial.transform import Rotation

K = np.array([[1055.3342285156250000, 0, 990.0682373046875], [0, 1055.334228515625, 544.24639892578125], [0, 0, 1]])
# K = np.array([[3558.1395, 0, 1224], [0, 3558.1395, 1024], [0, 0, 1]])
print(K)

# Image frame to Camera frame

# Image frame vector i
u = 1095  # on the table
v = 1099  # on the table
# u = 974 #zero vehicle
# v = 655 #zero vehicle
# u = 1206 #north from vehicle
# v = 1#north from vehicle
# u= 1316 #floor
# v = 139 #floor
# u = 1293  # top left
# v = 57
# u = 1391  # top right
# v = 55
# u = 1297  # bottom left
# v = 128
# u = 1396 #bottom right
# v = 126
# u = 1371 #under right
# v = 157
# u = 1281 #under left
# v = 154

i = np.array([[u], [v], [1]])

print('u, v')
print(i)

# Calculate Camera frame vector prime
P_C_prime = np.linalg.inv(K) @ i
print(f'P_C_prime: {P_C_prime}')

# Camera frame to Gimbal frame
T_C_to_G = np.array([[0], [0], [0]])
print(f'T_C_to_G: {T_C_to_G}')

R_C_to_G = Rotation.from_euler('xyz', [math.radians(90), 0, math.radians(90)]).as_matrix()
print(f'R_C_to_G: {R_C_to_G}')
# np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])

P_G_prime = R_C_to_G @ P_C_prime + T_C_to_G
print(f'P_G_prime: {P_G_prime}')

# Gimbal frame to UAS - IMU - frame
yaw = 0.00176  # rad
pitch = 0.00116  # rad
roll = 0.00138  # rad
# yaw = -math.pi / 2  # rad
# pitch = -math.pi / 3  # rad
# roll = 0  # rad

# from left_cam_imu_transform
# from the camera to the IMU of the ZED2: -0.002 -0.023 -0.002" rpy="0.00138 -0.00116 -0.00176

# yaw = yaw*math.pi/180 #rad
# pitch = pitch*math.pi/180 #rad
# roll = roll*math.pi/180 #rad
# print(math.degrees(yaw), math.degrees(pitch), math.degrees(roll))

T_G_to_UAS = np.array([[0.3], [0], [0.2]])
print(f'T_G_to_UAS: {T_G_to_UAS}')

R_G_to_UAS = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
print(f'R_G_to_UAS: {R_G_to_UAS}')

# T_G_to_UAS = np.array([[-0.002], [0.023], [0.002]])
P_UAS_prime = R_G_to_UAS @ P_G_prime + T_G_to_UAS
print(f'P_UAS_prime: {P_UAS_prime}')

# print(3 * P_UAS_prime)

# UAS frame - IMU - to NED frame. from imu/data then rotated to NED
# yaw = 6.156165920019568638 #rad
# pitch = -0.4100276152492891568 #rad
# roll = 0.02939145693704781417 #rad

yaw = 6.046293468769378
pitch = -1.466422693619277240
roll = -0.1061368580805083922
# yaw = 0
# pitch = 0
# roll = 0

print(math.degrees(yaw), math.degrees(pitch), math.degrees(roll))

T_UAS_to_NED = np.array([[0], [0], [0]])
print(f'T_UAS_to_NED: {T_UAS_to_NED}')

R_UAS_to_NED = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
print(f'R_UAS_to_NED: {R_UAS_to_NED}')

P_NED_prime = R_UAS_to_NED @ P_UAS_prime + T_UAS_to_NED

print(f'P_NED_prime:{P_NED_prime}')

# NED frame to ENU frame
R_NED_to_ENU = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
print(f'R_NED_to_ENU: {R_NED_to_ENU}')

UTM_x = 0
# UTM_x = 31.72212
# 581903.774

UTM_y = 0
# UTM_y = -6.55099
# 6582053.552

UTM_z = 8.88
# UTM_z = 42.44889

T_NED_to_ENU = np.array([[UTM_x], [UTM_y], [UTM_z]])

P_ENU_prime = R_NED_to_ENU @ P_NED_prime + T_NED_to_ENU

print(f'P_ENU_prime: {P_ENU_prime}')

# print(3 * (R_NED_to_ENU @ P_NED_prime) + T_NED_to_ENU)

# Calculating z_C
z_ENU = 0.85
# z_ENU = 0

#################
T = T_NED_to_ENU + R_NED_to_ENU @ T_UAS_to_NED + R_NED_to_ENU @ R_UAS_to_NED @ T_G_to_UAS + R_NED_to_ENU @ R_UAS_to_NED @ R_G_to_UAS @ T_C_to_G
print("T = ", T)
z_T = T[2]
print("z_T = ", z_T)
z_ENU_prime = P_ENU_prime[2]
print("z_ENU_prime = ", z_ENU_prime)
# print("z_T = ",z_T)
z_C = (z_ENU - z_T) / (z_ENU_prime - z_T)
print("z_C = ", z_C)

# Calculating P_ENU

P_ENU = z_C * P_ENU_prime - z_C * T + T
P_ENU = list(map(lambda P_ENU: str(P_ENU), P_ENU.round(4)))
print(P_ENU)
