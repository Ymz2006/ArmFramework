import numpy as np
import cv2 as cv
import cameras.realsense
import glob

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,5,0) for 9x6 board
square_size = 18  # 18 cm = 0.18 meters
objp = np.zeros((6*9, 3), np.float32)
objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2) * square_size


# Arrays to store object points and image points from all the images.
objpoints = []  # 3d points in real world space
imgpoints = []  # 2d points in image plane.


images = glob.glob('calibration_images/*.jpg')

image_cnt = 0
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners for a 9x6 pattern
    ret, corners = cv.findChessboardCorners(gray, (9, 6), None)

    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)

        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv.drawChessboardCorners(img, (9, 6), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(50)
        cv.destroyAllWindows()

        image_cnt += 1

print("image count: " + str(image_cnt))

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)


camera = cameras.realsense.Camera("camera")
camera.init_camera()

img = camera.get_frame()
h,  w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))


# undistort
dst = cv.undistort(img, mtx, dist, None, newcameramtx)
 
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv.imwrite('calibration_images/calibresult1.png', dst)


fs_write = cv.FileStorage('camera_calib.yaml', cv.FILE_STORAGE_WRITE)

fs_write.write('camera_matrix', mtx)
fs_write.write('dist_coeff', dist)
fs_write.write('new_camera_matrix', newcameramtx)

fs_write.release()




