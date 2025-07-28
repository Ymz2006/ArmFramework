import cv2 as cv
import numpy as np
import src.ec_robot as ec_robot

click_pos = None

def mouse_click(event, x, y, flags, param):
    global click_time, click_pos, show_circle
    if event == cv.EVENT_LBUTTONDOWN:
        click_pos = (x, y)
        show_circle = True
        print(f"Mouse clicked at: ({x}, {y})")



fs_read = cv.FileStorage('camera_calib.yaml', cv.FILE_STORAGE_READ)
camera_matrix = fs_read.getNode('camera_matrix').mat()
dist_coeff = fs_read.getNode('dist_coeff').mat()
new_camera_matrix = fs_read.getNode('new_camera_matrix').mat()
fs_read.release()

#need to init img
img = cv.imread('testpic.jpg')
undistorted = cv.undistort(img, camera_matrix, dist_coeff, None, new_camera_matrix)


fx = camera_matrix[0, 0]
fy = camera_matrix[1, 1]
cx = camera_matrix[0, 2]
cy = camera_matrix[1, 2]

cv.namedWindow('Undistorted Image')
cv.setMouseCallback('Undistorted Image', mouse_click)
while True:
    img_display = undistorted.copy()

    if click_pos:
        cv.circle(img_display, click_pos, 5, (0, 255, 0), -1)

    cv.imshow('Undistorted Image', img_display)

    key = cv.waitKey(1)
    if key == ord('q'):
        break

cv.destroyAllWindows()
u, v = click_pos



Z = 230.14  

# Compute 3D point in camera coordinate system
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy

point_3d = np.array([X, Y, Z])
print(point_3d)



robot = ec_robot.ECRobot("ec_robot")
    
curr_pos = robot.read_current_pose()
print(curr_pos)
robot.set_pose([0.1, 0, 0.5, 0, 0, 0] + [X, Y, 0, 0, 0, 0])
curr_pos = robot.read_current_pose()
print(curr_pos)

