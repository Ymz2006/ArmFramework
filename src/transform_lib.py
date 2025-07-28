import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation 
from mpl_toolkits.mplot3d import Axes3D
import math 

def rotation_matrix_to_euler2(mat, degrees):
    r = Rotation.from_matrix(mat)
    euler_angles = r.as_euler('xyz', degrees=degrees)
    return euler_angles

def euler_to_rotation_matrix2(roll, pitch, yaw, degrees = True):
    r = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=degrees)
    rotation_matrix = r.as_matrix()
    return rotation_matrix


def camera_2d_rot(theta,x, y):
    rot = np.array([
    [np.cos(theta), -np.sin(theta)],
    [np.sin(theta), np.cos(theta)]])
    return rot@np.array[[x],[y]]



def getTransformation(pose):
    return pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]

def getHomogenousTransformationMatrix(pose):
    x,y,z,rx,ry,rz = getTransformation(pose)
    r = euler_to_rotation_matrix2(rx,ry,rz)
    t = np.array([x, y, z])  # Translation

    transformation = np.vstack([
    np.hstack([r, t.reshape(3, 1)]),
    [0, 0, 0, 1]
    ])
    return transformation

def getCoordinatesFromMatrix(matrix, degrees):
    r = matrix[:3, :3]         
    t = matrix[:3, 3]          
    rx, ry, rz = rotation_matrix_to_euler2(r, degrees=degrees)
    return ( t[0], t[1], t[2], rx, ry, rz)
    

    