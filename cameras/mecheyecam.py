import numpy as np
import cv2

from mecheye.shared import *
from mecheye.area_scan_3d_camera import *
from mecheye.area_scan_3d_camera_utils import find_and_connect

class MecheyeCam():
    def __init__(self):
        self.camera_status = False
        self.camera = Camera()

    def init_camera(self):
        self.camera_status = find_and_connect(self.camera)
        
        return self.camera_status
    
    def get_frame(self):
        '''
            output:
                frame: np.array
        '''
        if self.camera_status:
            frame_2d = Frame2D()
            show_error(self.camera.capture_2d(frame_2d))
            if frame_2d.color_type() == ColorTypeOf2DCamera_Monochrome:
                image2d = frame_2d.get_gray_scale_image()
            elif frame_2d.color_type() == ColorTypeOf2DCamera_Color:
                image2d = frame_2d.get_color_image()
            frame = image2d.data()
            return frame
        else:
            print("Camera is closed!")
            return None
        
    def camera_close(self):
        # Stop streaming
        self.camera.disconnect()
        print("Disconnected from the camera successfully.")

if __name__ == "__main__":
    cam = Mecheye()
    data = cam.get_frame()
    file_name = "2DImage.png"
    cv2.imwrite(file_name, data)
    cam.camera_close()


"""
Connect Mech-Eye Industrial 3D Camera Successfully.
Texture Camera Matrix: 
    [2744.1906043165463, 0, 971.8695521366607]
    [0, 2743.3731471296705, 603.8157771046684]
    [0, 0, 1]

Texture Camera Distortion Coefficients: 
    k1: 0.0, k2: 0.0, p1: 0.0, p2: 0.0, k3: 0.0

Depth Camera Matrix: 
    [2744.1906043165463, 0, 971.8695521366607]
    [0, 2743.3731471296705, 603.8157771046684]
    [0, 0, 1]

Depth Camera Distortion Coefficients: 
    k1: 0.0, k2: 0.0, p1: 0.0, p2: 0.0, k3: 0.0

Rotation: From Depth Camera to Texture Camera: 
    [1.0, 0.0, 0.0]
    [0.0, 1.0, 0.0]
    [0.0, 0.0, 1.0]

Translation From Depth Camera to Texture Camera: 
    X: 0.0mm, Y: 0.0mm, Z: 0.0mm
"""