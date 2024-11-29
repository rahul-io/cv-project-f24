import cv2
import numpy as np
import glob

# Defining the dimensions of the checkerboard
CHECKERBOARD = (8, 6)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Creating vectors to store 3D and 2D points
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Defining the world coordinates for 3D points
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# Extracting paths of images
images = glob.glob("C:/Users/ghiya/Downloads/camera_calibration/Data/*.jpg")

if not images:
    print("No images found. Check the directory path.")
    exit()

# Loop over all images to detect the chessboard corners
for fname in images:
    img = cv2.imread(fname)
    img = cv2.resize(img, (1024, int(1024 * img.shape[0] / img.shape[1])))  # Resize for better display
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,
                                             cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if ret:
        print(f"Chessboard corners detected in: {fname}")
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display corners
        img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)  # Display for 500ms
    else:
        print(f"Chessboard corners NOT detected in: {fname}")

cv2.destroyAllWindows()

# Perform camera calibration
if len(objpoints) < 2:
    print("Not enough valid images for calibration. At least 2 are required.")
    exit()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Print results
print("\nCamera matrix:")
print(mtx)

print("\nDistortion coefficients:")
print(dist)

print("\nRotation vectors:")
print(rvecs)

print("\nTranslation vectors:")
print(tvecs)

# Step 1: Calculate Reprojection Error
total_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    total_error += error

mean_error = total_error / len(objpoints)
print(f"\nReprojection Error: {mean_error}")

# Step 2: Undistort an image to verify calibration
test_img = cv2.imread(images[0])  # Use the first calibration image
h, w = test_img.shape[:2]

# Get optimal new camera matrix
new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

# Undistort the image
undistorted_img = cv2.undistort(test_img, mtx, dist, None, new_camera_mtx)

# Crop the image based on the ROI
x, y, w, h = roi
undistorted_img = undistorted_img[y:y+h, x:x+w]

# Show original and undistorted images
cv2.imshow("Original Image", test_img)
cv2.imshow("Undistorted Image", undistorted_img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Step 3: Project 3D points to 2D and compare with the detected corners
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)

    # Draw the reprojected points on the image
    img = cv2.imread(images[i])
    img = cv2.resize(img, (1024, int(1024 * img.shape[0] / img.shape[1])))

    for p in imgpoints2:
        cv2.circle(img, tuple(p.ravel()), 5, (0, 255, 0), -1)

    # Display the image with reprojected points
    cv2.imshow(f"Reprojected Points - Image {i+1}", img)
    cv2.waitKey(0)

cv2.destroyAllWindows()
