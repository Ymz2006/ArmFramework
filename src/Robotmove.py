import cv2
import ec_robot
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation 
#from cameras.realsense import Realsense
from mpl_toolkits.mplot3d import Axes3D
import math 
import transform_lib as transform
import visualize_lib as visualize



class Robotmove():
    
    arm_to_robot_z = 10 #in degrees 
    camera_to_arm_rotation = 0 #in degrees  

    robotToArmMatrix = transform.getHomogenousTransformationMatrix([0,0,0,0,0,-30])
    

    def __init__(self):
        self.robot = None #ec_robot.ECRobot("ec_robot")
        self.realsense = None #Realsense("RealsenseCamera")


    def moveRobot(self,pose):
        self.robot.set_pose(pose)
    
    def printPose(self):
        print(self.robot.read_current_pose)


    #input is [x,y,z,rx,ry,rz]
    def robotToArm(self,pose):
        return transform.getHomogenousTransformationMatrix(pose) @ self.robotToArmMatrix # should be left multiply inverse but is the same result
        

    
    def moveCameraAruco(self, id):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

        while True:
            ret, image = self.realsense.get_frame()
            height, width = image.shape[:2]
            if not ret:
                print("Failed to capture image")
                break

            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = detector.detectMarkers(gray)

            # Initialize variables for ID=1
            marker_found = False
            midpoint = None

            if ids is not None:
                for i, marker_id in enumerate(ids):
                    if marker_id == 42:  # Only process ID=1
                        marker_corners = corners[i][0]
                        
                        # Calculate midpoint (average of all corners)
                        midpoint = np.mean(marker_corners, axis=0).astype(int)
                        print(f"Midpoint of ID=42: {midpoint}")
                        marker_found = True
                        
                        # Draw marker and midpoint
                        cv2.aruco.drawDetectedMarkers(image, [corners[i]], np.array([[marker_id]]))

                        cv2.circle(image, tuple(midpoint), 5, (0, 255, 0), -1)
                        cv2.putText(image, f"Mid: {midpoint}", tuple(midpoint + 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        

                        xmid,ymid = midpoint
                        dy = height/2 - ymid
                        dx = width/2 - xmid

                        axis_dir = transform.camera_2d_rot@np.array[[dx],[dy]] 
                        
                        r = math.sqrt(dx*dx + dy*dy)
                        dx = dx/r
                        dy = dy/r


                        #set robot move here 
                        

            if not marker_found:
                print("ID=42 not found")

            cv2.imshow('ArUco Detection', image)
            
            # Exit on 'q' key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

            
    def undistortImage(self,img):
        fs_read = cv2.FileStorage('camera_params\\camera_calib.yaml', cv2.FILE_STORAGE_READ)
        camera_matrix = fs_read.getNode('camera_matrix').mat()
        dist_coeff = fs_read.getNode('dist_coeff').mat()
        new_camera_matrix = fs_read.getNode('new_camera_matrix').mat()
        fs_read.release()

        undistorted = cv2.undistort(img, camera_matrix, dist_coeff, None, new_camera_matrix)
        return undistorted
    
    


robotmove = Robotmove()
robotmove.undistortImage(cv2.imread('aruco_markers\\marker0.jpg'))


T2 = transform.getHomogenousTransformationMatrix([0,0,0,0,0,30]) # arm in robot perspective 
T2coordinates = transform.getCoordinatesFromMatrix(T2, degrees=True)

T3 = transform.getHomogenousTransformationMatrix([0,5,1,0,-45,0])
T3coordinates = transform.getCoordinatesFromMatrix(T3, degrees=True)



T2inverse = transform.getHomogenousTransformationMatrix([0,0,0,0,0,-30]) # robot in arm perspective 

T2system = T3@T2inverse

also = T2system@T2

alsoc = transform.getCoordinatesFromMatrix(also, degrees=True)

frames = [
    {'x': 0, 'y': 0, 'z': 0, 'roll': 0, 'pitch': 0, 'yaw': 0, 'label': 'robot frame'},

    {'x': T2coordinates[0], 'y': T2coordinates[1], 'z': T2coordinates[2], 'roll': T2coordinates[3], 
     'pitch': T2coordinates[4], 'yaw': T2coordinates[5], 'label': 'arm frame'},
    

    {'x': T3coordinates[0], 'y': T3coordinates[1], 'z': T3coordinates[2], 'roll': T3coordinates[3], 
     'pitch': T3coordinates[4], 'yaw': T3coordinates[5], 'label': 'end affector'},

     
    {'x': alsoc[0], 'y': alsoc[1], 'z': alsoc[2], 'roll': alsoc[3], 
     'pitch': alsoc[4], 'yaw': alsoc[5], 'label': 'also end'},
]



visualize.plot_multiple_frames(frames, length=0.5, degrees=True)

