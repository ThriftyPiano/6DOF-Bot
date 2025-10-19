import numpy as np
import cv2
import glob
import os

def calibrate_camera(chessboard_size=(9, 6), square_size=0.025, num_images=10, save_path="calibration_data/"):
    """
    Performs camera calibration using a chessboard pattern.

    Args:
        chessboard_size (tuple): Dimensions of the chessboard (width, height).
                                 e.g., (9, 6) for a 9x6 inner corner grid.
        square_size (float): Size of a single square on the chessboard in meters.
                             This is crucial for real-world measurements.
        num_images (int): Number of images to capture for calibration.
        save_path (str): Directory to save calibration data and captured images.
    """

    # Ensure the save path exists
    if not os.path.exists(save_path):
        os.makedirs(save_path)
        print(f"Created directory: {save_path}")

    # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)
    # These are the 3D coordinates of the chessboard corners in its own coordinate system.
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp = objp * square_size # Scale by actual square size

    # Arrays to store object points and image points from all images
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.


    # Start video capture
    cap = cv2.VideoCapture(0) # 0 for default camera

    if not cap.isOpened():
        print("Error: Could not open video stream.")
        return

    # Set capture resolution instead of resizing later
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print("Camera Calibration Mode")
    print("Press 's' to save an image with detected corners.")
    print("Press 'e' to end and calibrate (do not use 'q').")

    cameraMatrix = np.array([[851.1366412, 0, 759.71772254],
                             [0, 866.62577366, 516.37335944],
                             [0, 0, 1]], dtype=np.float32)
    distCoeffs = np.array([-0.30494122, 0.10215236, 0.00597334, 0.01665244, -0.01335692], dtype=np.float32)

    captured_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break
        # Frame is now captured at the desired resolution
        frame = cv2.undistort(frame, cameraMatrix, distCoeffs)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        # The `cv2.CALIB_CB_ADAPTIVE_THRESH` flag helps with varying lighting.
        # `cv2.CALIB_CB_NORMALIZE_IMAGE` normalizes brightness and contrast.
        ret_corners, corners = cv2.findChessboardCorners(gray, chessboard_size, None,
                                                         cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

        # If found, add object points, image points (after refining them)
        if ret_corners:
            # Refine corner positions for increased accuracy
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # Draw and display the corners
            cv2.drawChessboardCorners(frame, chessboard_size, corners2, ret_corners)
            cv2.putText(frame, f"Corners Found! Press 's' to save.", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        else:
            cv2.putText(frame, "No Chessboard Found. Adjust position.", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.putText(frame, f"Captured: {captured_count}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow('Camera Calibration - Press \'s\' to Save, \'e\' to End', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s') and ret_corners:
            # Save the image and add points if corners are found
            img_name = os.path.join(save_path, f"calibration_image_{captured_count:02d}.png")
            cv2.imwrite(img_name, frame)
            objpoints.append(objp)
            imgpoints.append(corners2)
            captured_count += 1
            print(f"Image saved: {img_name}")
            print(f"Total images captured: {captured_count}")
            # Give a small delay to avoid capturing the same frame multiple times
            cv2.waitKey(500)
        elif key == ord('e'):
            print("Calibration process ended by user.")
            break

    # Release the camera
    cap.release()
    cv2.destroyAllWindows()


    if captured_count < 3:
        print("Not enough images captured for calibration. Please try again (minimum 3 required).")
        return

    print("\nStarting camera calibration...")
    # Perform camera calibration
    # ret: True if calibration successful
    # mtx: Camera matrix (intrinsic parameters)
    # dist: Distortion coefficients
    # rvecs: Rotation vectors for each image
    # tvecs: Translation vectors for each image
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    if ret:
        print("\nCalibration successful!")
        print("\nCamera Matrix (mtx):\n", mtx)
        print("\nDistortion Coefficients (dist):\n", dist)

        # Save the calibration results
        np.save(os.path.join(save_path, "camera_matrix.npy"), mtx)
        np.save(os.path.join(save_path, "dist_coeffs.npy"), dist)
        print(f"\nCalibration data saved to {save_path}camera_matrix.npy and {save_path}dist_coeffs.npy")

        # Optional: Calculate re-projection error
        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error
        print(f"\nTotal re-projection error: {mean_error/len(objpoints)}")

        # Optional: Demonstrate undistortion on a new image
        print("\nPress any key to see undistorted image (if available).")
        # Try to load the last captured image for demonstration
        if captured_count > 0:
            test_img_path = os.path.join(save_path, f"calibration_image_{captured_count-1:02d}.png")
            if os.path.exists(test_img_path):
                img = cv2.imread(test_img_path)
                h, w = img.shape[:2]
                newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

                # Undistort
                dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

                # Crop the image (optional, based on ROI)
                x, y, w, h = roi
                dst = dst[y:y+h, x:x+w]

                cv2.imshow('Original Image', img)
                cv2.imshow('Undistorted Image', dst)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            else:
                print("Could not find a captured image for undistortion demonstration.")
        else:
            print("No images were captured to demonstrate undistortion.")

    else:
        print("Camera calibration failed. Please ensure good lighting and clear chessboard views.")

if __name__ == "__main__":
    # Define your chessboard dimensions (inner corners)
    # For a 9x6 chessboard, there are 8x5 inner corners.
    # So, width=9, height=6 refers to the number of corners along each dimension.
    # Standard 9x6 chessboard has 9 corners horizontally and 6 corners vertically.
    CHESSBOARD_WIDTH = 9
    CHESSBOARD_HEIGHT = 6
    SQUARE_SIZE_METERS = 0.025 # Example: 2.5 cm per square. Measure your actual chessboard!
    NUM_CALIBRATION_IMAGES = 15 # Aim for 10-20 good images

    calibrate_camera(chessboard_size=(CHESSBOARD_WIDTH, CHESSBOARD_HEIGHT),
                     square_size=SQUARE_SIZE_METERS,
                     num_images=NUM_CALIBRATION_IMAGES)
