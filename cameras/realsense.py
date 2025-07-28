from .base_camera import Camera
import pyrealsense2 as rs
import numpy as np
class Realsense(Camera):
    def __init__(self,name):
        super().__init__(name)
        self.camera_status = False
    def init_camera(self):
        #init realsense
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        print(device)
        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)
        
        serial_number = "335622070497"  # 替换为你要使用的相机的序列号 #手部相机：335622070497   底座相机： 327122071506 
        config.enable_device(serial_number)

        config.enable_stream(rs.stream.depth,640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color,640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(config)
        self.camera_status = True

    def get_frame(self):
        '''
            output:
                frame: np.array
        '''
        if self.camera_status:
            # 拍摄标定板图像并保存
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            # Convert images to numpy arrays
            frame = np.asanyarray(color_frame.get_data())
            return frame
        else:
            print("Camera is closed!")
            return None
        
    def camera_close(self):
        # Stop streaming
        self.pipeline.stop()
        self.camera_status = False
