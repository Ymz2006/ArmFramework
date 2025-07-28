import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation 
from mpl_toolkits.mplot3d import Axes3D
import math 
import transform_lib as transform

def set_axes_equal(ax):
    """Set equal scaling for all 3D axes (so X, Y, Z units appear equal)."""
    limits = np.array([ax.get_xlim(), ax.get_ylim(), ax.get_zlim()])
    centers = np.mean(limits, axis=1)
    max_range = np.max(limits[:, 1] - limits[:, 0])
    half_range = max_range / 2

    for i, axis in enumerate([ax.set_xlim, ax.set_ylim, ax.set_zlim]):
        axis([centers[i] - half_range, centers[i] + half_range])


def plot_frame(ax, x, y, z, roll, pitch, yaw, length=1.0, degrees=True, label='F'):
    """Plot one local coordinate frame on given axes."""
    origin = np.array([x, y, z])
    R = transform.euler_to_rotation_matrix2(roll, pitch, yaw, degrees)

    colors = ['r', 'g', 'b']
    axes_labels = ['X', 'Y', 'Z']

    for i in range(3):
        direction = R[:, i] * length
        ax.quiver(*origin, *direction, color=colors[i], arrow_length_ratio=0.1)
        ax.text(*(origin + direction * 1.1), f'{label}_{axes_labels[i]}', color=colors[i])


def plot_frame(ax, x, y, z, roll, pitch, yaw, length=1.0, degrees=True, label='F'):
    """Plot one local coordinate frame on given axes."""
    origin = np.array([x, y, z])
    R = transform.euler_to_rotation_matrix2(roll, pitch, yaw, degrees)

    colors = ['r', 'g', 'b']
    axes_labels = ['X', 'Y', 'Z']

    for i in range(3):
        direction = R[:, i] * length
        ax.quiver(*origin, *direction, color=colors[i], arrow_length_ratio=0.1)
        ax.text(*(origin + direction * 1.1), f'{label}_{axes_labels[i]}', color=colors[i])


def plot_multiple_frames(frames, length=1.0, degrees=True):
    """
    Plot multiple coordinate frames in a single 3D plot.
    
    Parameters:
        frames: List of dicts with keys:
            - x, y, z
            - roll, pitch, yaw
            - label (optional)
        length: Arrow length for each axis
        degrees: Set True if angles are in degrees
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for frame in frames:
        plot_frame(
            ax,
            x=frame['x'],
            y=frame['y'],
            z=frame['z'],
            roll=frame['roll'],
            pitch=frame['pitch'],
            yaw=frame['yaw'],
            length=length,
            degrees=degrees,
            label=frame.get('label', '')
        )

    # Auto scale and fix aspect ratio
    all_positions = np.array([[f['x'], f['y'], f['z']] for f in frames])
    min_vals = np.min(all_positions, axis=0) - length * 1.5
    max_vals = np.max(all_positions, axis=0) + length * 1.5
    ax.set_xlim([min_vals[0], max_vals[0]])
    ax.set_ylim([min_vals[1], max_vals[1]])
    ax.set_zlim([min_vals[2], max_vals[2]])
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Multiple Local Coordinate Frames')
    ax.grid(True)
    
    set_axes_equal(ax)
    plt.show()
