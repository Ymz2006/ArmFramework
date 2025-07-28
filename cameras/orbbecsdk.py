from .base_camera import Camera
from pyorbbecsdk import Pipeline, FrameSet
from pyorbbecsdk import Config
from pyorbbecsdk import OBSensorType, OBFormat
from pyorbbecsdk import OBError
from pyorbbecsdk import VideoStreamProfile
import cv2
import numpy as np

class Orbbecsdk(Camera):
    def __init__(self,name):
        super().__init__(name)
        self.camera_status = False
    def init_camera(self):
        #init ob
        config = Config()
        pipeline = Pipeline()
        try:
            profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
            try:
                color_profile: VideoStreamProfile = profile_list.get_video_stream_profile(640, 0, OBFormat.RGB, 30)
            except OBError as e:
                print(e)
                color_profile = profile_list.get_default_video_stream_profile()
            config.enable_stream(color_profile)
        except Exception as e:
            print(e)
            return
        pipeline.start(config)
        self.camera_status = True

    def get_frame(self):
        '''
            output:
                frame: np.array
        '''
        if self.camera_status:
            frames: FrameSet = self.pipeline.wait_for_frames(100)
            color_frame = frames.get_color_frame()
            # covert to RGB format
            def get_np_image(frame):
                width = frame.get_width()
                height = frame.get_height()
                color_format = frame.get_format()
                data = np.asanyarray(frame.get_data())
                image = np.resize(data, (height, width, 3))
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                return image
            frame = get_np_image(color_frame)
            return frame
        else:
            print("Camera is closed!")
            return None
        
    def camera_close(self):
        # Stop streaming
        self.pipeline.stop()
        self.camera_status = False
