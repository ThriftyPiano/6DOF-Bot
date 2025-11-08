#!/usr/bin/env python3
import cv2
import numpy as np
import serial
import serial.tools.list_ports
import time
import sys
import glob
from typing import Tuple, List, Optional, Union

# Global variables
servo = serial.Serial('/dev/tty.usbserial-1120', 115200, timeout=0.5)
servoPos = 0
lastCalculatedRelativeAngle = None

def init_camera() -> Tuple[cv2.VideoCapture, int, int]:
    """Initialize the camera with specified resolution."""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera")
        sys.exit(1)
    
    # Set to full resolution for better undistortion
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera resolution set to: {width}x{height}")
    return cap, width, height

def get_calibration_matrices() -> Tuple[np.ndarray, np.ndarray]:
    """Initialize camera calibration parameters."""
    camera_matrix = np.array([
        [786.357, 0, 989.731],
        [0, 785.863, 501.663],
        [0, 0, 1]
    ], dtype=np.float32)
    
    dist_coeffs = np.array([
        -0.370767, 0.106516, 0.000118, -0.000542, -0.012277
    ], dtype=np.float32)
    
    return camera_matrix, dist_coeffs

def undistort_and_scale(frame: np.ndarray, camera_matrix: np.ndarray, 
                       dist_coeffs: np.ndarray, target_size: Tuple[int, int]) -> np.ndarray:
    """Undistort the frame and scale to target size."""
    undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
    return cv2.resize(undistorted, target_size)

def detect_color_contours(frame: np.ndarray, color: str) -> Tuple[List[np.ndarray], np.ndarray]:
    """Detect contours for specified color (red or blue) in the frame."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    if color == "red":
        # Red has two ranges in HSV
        lower_red1 = (170, 140, 70)
        upper_red1 = (180, 255, 225)
        lower_red2 = (0, 140, 70)
        upper_red2 = (10, 255, 225)
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
    else:  # blue
        lower_blue = (105, 140, 70)
        upper_blue = (115, 255, 225)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    # Apply morphological operations
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filtered_frame = cv2.bitwise_and(frame, frame, mask=mask)
    
    return contours, filtered_frame

def find_highest_point(contour: np.ndarray) -> Optional[Tuple[tuple, float]]:
    """Find the highest point in a contour."""
    contour_points = contour.squeeze()
    if len(contour_points.shape) >= 2:
        min_y_idx = np.argmin(contour_points[:, 1])
        highest_point = tuple(contour_points[min_y_idx])
        return highest_point, float(highest_point[1])
    return None

def process_contours(contours: List[np.ndarray], min_area: float) -> Tuple[List[np.ndarray], Optional[Tuple[tuple, float]]]:
    """Process contours to find valid ones and the highest point."""
    valid_contours = []
    highest_point = None
    highest_y = float('inf')
    
    for contour in contours:
        if cv2.contourArea(contour) >= min_area:
            valid_contours.append(contour)
            result = find_highest_point(contour)
            if result is not None:
                point, y = result
                if y < highest_y:
                    highest_y = y
                    highest_point = point
    
    return valid_contours, (highest_point, highest_y) if highest_point is not None else None

def draw_results(frame: np.ndarray, contours: List[np.ndarray], 
                highest_point: Optional[Tuple[tuple, float]], 
                color: Tuple[int, int, int]) -> None:
    """Draw contours and highest point on the frame."""
    if contours:
        cv2.drawContours(frame, contours, -1, color, 2)
        if highest_point is not None:
            point, y = highest_point
            cv2.circle(frame, point, 5, (0, 255, 0), -1)
            cv2.putText(frame, f"Highest: {y:.1f}px", 
                      (point[0], point[1] - 10),
                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

def add_frame_info(frame: np.ndarray, fps: float, width: int, height: int, label: str) -> None:
    """Add FPS, resolution, and label information to the frame."""
    cv2.putText(frame, f"FPS: {fps:.1f}", (30, 50), 
               cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(frame, f"Resolution: {width}x{height}", (30, 100),
               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    if label:
        cv2.putText(frame, label, (30, 150),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

# Calculate relative angle of point based on focal length
def calculate_relative_angle(point: Tuple[int, int], width: int, focal_length: float = 131.06) -> float:
    """Calculate the relative angle of a point using the camera's focal length.
    
    Args:
        point: The (x,y) coordinates of the point
        width: The width of the frame
        focal_length: Camera's focal length in pixels (default: fx from calibration)
    
    Returns:
        angle: The angle in degrees from the center
    """
    x, _ = point
    # Use camera's principal point (cx) which is 320/2 = 160 for our scaled image
    center_x = width  # This is cx from the camera matrix for 320x240 resolution
    # Calculate angle using arctangent of (x offset / focal length)
    angle = np.arctan2(x - center_x, focal_length)
    return np.degrees(angle)

# Move servo such that the highest detected point is centered
def move_servo(highest_point: Optional[Tuple[tuple, float]], width: int, height: int) -> None:
    """Move servo to center the highest detected point."""
    global servo
    
    if highest_point is not None:
        point, y = highest_point
        # Calculate angle using the camera's focal length and calibration
        angle = calculate_relative_angle(point, width)
        
        # Convert angle to servo value using the servo's range
        # servoTurnRate = 135/0.9 (degrees per servo value)
        # We need to convert our measured angle to a servo value
        global servoPos
        new_servo_value = servoPos - angle / (135 / 0.9)  # Convert degrees to servo value

        global lastCalculatedRelativeAngle

        # Prevent duplicate movements
        if lastCalculatedRelativeAngle is not None:
            if abs(lastCalculatedRelativeAngle - angle) < 5:
                return

        lastCalculatedRelativeAngle = angle
        
        # Bound the servo value between -0.9 and 0.9
        new_servo_value = max(-0.9, min(0.9, new_servo_value))
        
        # Check if the new position is significantly different from current position
        # 5 degrees = 5 / (135/0.9) ≈ 0.033 in servo value units
        if abs(new_servo_value - servoPos) > 0.005:
            servoPos = new_servo_value  # Update global position
            
            print(f"Calculated angle: {angle:.2f}")
            print(f"Servo value: {servoPos:.2f}")
            
            # Send the command to the servo with the correct format
            servo.write(f'set_servo_position(5, {servoPos})\r'.encode())
        
    servo.close()

def main():
    # Initialize camera and calibration
    cap, orig_width, orig_height = init_camera()
    camera_matrix, dist_coeffs = get_calibration_matrices()
    target_size = (320, 240)
    
    # FPS calculation variables
    fps_counter = 0
    fps_start_time = time.time()
    fps_display = 0
    MIN_CONTOUR_AREA = 100  # Base minimum area for 320x240
    
    # Tracking parameters
    TRACK_COLOR = "red"  # Can be "red" or "blue"
    auto_aim_enabled = True  # State for auto-aiming
    
    print(f"Tracking {TRACK_COLOR} objects")
    print("Press 'q' to quit, 'ESC' to exit")
    print("Press 'r' to track red, 'b' to track blue")
    print("Press SPACE to toggle auto-aim")

    # Initialize servo to center position
    servo = serial.Serial('/dev/tty.usbserial-1120', 115200, timeout=0.5)
    servo.write(b'set_servo_position(5, 0)\r')
    time.sleep(0.5)  # Give servo time to move to initial position
    servo.close()

    while True:
        cv2.waitKey(500)
        # Capture and process frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to grab frame")
            break
            
        # Undistort and scale frame
        processed_frame = undistort_and_scale(frame, camera_matrix, dist_coeffs, target_size)
        
        # Process red contours
        red_contours, red_filtered = detect_color_contours(processed_frame, "red")
        valid_red_contours, red_highest = process_contours(red_contours, MIN_CONTOUR_AREA)
        draw_results(red_filtered, valid_red_contours, red_highest, (0, 0, 255))
        
        # Process blue contours
        blue_contours, blue_filtered = detect_color_contours(processed_frame, "blue")
        valid_blue_contours, blue_highest = process_contours(blue_contours, MIN_CONTOUR_AREA)
        draw_results(blue_filtered, valid_blue_contours, blue_highest, (255, 0, 0))
        
        # Calculate FPS
        fps_counter += 1
        elapsed_time = time.time() - fps_start_time
        if elapsed_time >= 1.0:
            fps_display = fps_counter / elapsed_time
            fps_counter = 0
            fps_start_time = time.time()
        
        # Add information to frames
        width, height = target_size
        add_frame_info(red_filtered, fps_display, width, height, "Red Filter")
        add_frame_info(blue_filtered, fps_display, width, height, "Blue/Cyan Filter")
        
        # Update servo position based on tracked color if auto-aim is enabled
        current_time = time.time()
        if auto_aim_enabled:
            if TRACK_COLOR == "red":
                move_servo(red_highest, target_size[0], target_size[1])
            else:
                move_servo(blue_highest, target_size[0], target_size[1])
            last_movement_time = current_time

        # Display frames
        cv2.imshow('Raw Camera Stream', cv2.resize(frame, target_size))
        cv2.imshow('Undistorted Stream', processed_frame)
        cv2.imshow('Red Filtered Stream', red_filtered)
        cv2.imshow('Blue/Cyan Filtered Stream', blue_filtered)
        
        # Add tracking and auto-aim indicators
        status_text = f"Tracking: {TRACK_COLOR} | Auto-aim: {'ON' if auto_aim_enabled else 'OFF'}"
        cv2.putText(processed_frame, status_text, (30, 150),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        
        # Check for exit, color switching, and auto-aim toggle
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:  # 27 is ESC key
            break
        elif key == ord('r'):
            TRACK_COLOR = "red"
            print("Switching to track red")
        elif key == ord('b'):
            TRACK_COLOR = "blue"
            print("Switching to track blue")
        elif key == ord(' '):  # Space bar
            auto_aim_enabled = not auto_aim_enabled
            print(f"Auto-aim {'enabled' if auto_aim_enabled else 'disabled'}")
    
    # Clean up
    cap.release()
    cv2.destroyAllWindows()
    print("Camera stream ended")

if __name__ == "__main__":
    main()