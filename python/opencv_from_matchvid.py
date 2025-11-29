#!/usr/bin/env python3
import cv2
import numpy as np
import time
import sys
import threading
from typing import Tuple, List, Optional

# Global variables for threading
angle_lock = threading.Lock()
current_angle = None
frame_lock = threading.Lock()
display_frames = {}
tracking_color = "red"
should_exit = False
video_paused = False

def init_video(video_path: str) -> Tuple[cv2.VideoCapture, int, int, float]:
    """Initialize the video capture with the match video."""
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Could not open video file: {video_path}")
        sys.exit(1)
    
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    print(f"Video resolution: {width}x{height}")
    print(f"Video FPS: {fps}")
    print(f"Total frames: {frame_count}")
    print(f"Duration: {frame_count/fps:.1f} seconds")
    
    return cap, width, height, fps

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

def add_frame_info(frame: np.ndarray, fps: float, width: int, height: int, label: str, 
                   frame_num: int = 0, total_frames: int = 0, timestamp: float = 0) -> None:
    """Add FPS, resolution, label, and video progress information to the frame."""
    cv2.putText(frame, f"FPS: {fps:.1f}", (30, 20), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(frame, f"Resolution: {width}x{height}", (30, 40),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(frame, f"Frame: {frame_num}/{total_frames}", (30, 60),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(frame, f"Time: {timestamp:.1f}s", (30, 80),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
    if label:
        cv2.putText(frame, label, (30, 100),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)

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
    center_x = width / 2
    angle = np.arctan2(x - center_x, focal_length)
    return np.degrees(angle)

class VideoProcessingThread(threading.Thread):
    """Separate thread for video processing and angle calculation."""
    
    def __init__(self, cap, camera_matrix, dist_coeffs, target_size, video_fps):
        threading.Thread.__init__(self)
        self.cap = cap
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.target_size = target_size
        self.video_fps = video_fps
        self.daemon = True
        
        # Video control variables
        self.frame_delay = 1.0 / video_fps if video_fps > 0 else 1.0 / 30.0
        self.total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.current_frame_num = 0
        
        # Processing variables
        self.MIN_CONTOUR_AREA = 100
        
    def run(self):
        """Main video processing loop."""
        global current_angle, display_frames, tracking_color, should_exit, video_paused
        
        print("Video processing thread started")
        
        while not should_exit:
            if video_paused:
                time.sleep(0.1)
                continue
                
            # Capture frame from video
            ret, frame = self.cap.read()
            if not ret:
                # End of video - loop back to beginning
                print("End of video reached, looping...")
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                self.current_frame_num = 0
                continue
                
            self.current_frame_num = int(self.cap.get(cv2.CAP_PROP_POS_FRAMES))
            timestamp = self.current_frame_num / self.video_fps
            
            # Undistort and scale frame
            processed_frame = undistort_and_scale(frame, self.camera_matrix, self.dist_coeffs, self.target_size)
            
            # Process red contours
            red_contours, red_filtered = detect_color_contours(processed_frame, "red")
            valid_red_contours, red_highest = process_contours(red_contours, self.MIN_CONTOUR_AREA)
            draw_results(red_filtered, valid_red_contours, red_highest, (0, 0, 255))
            
            # Process blue contours
            blue_contours, blue_filtered = detect_color_contours(processed_frame, "blue")
            valid_blue_contours, blue_highest = process_contours(blue_contours, self.MIN_CONTOUR_AREA)
            draw_results(blue_filtered, valid_blue_contours, blue_highest, (255, 0, 0))
            
            # Add information to frames
            width, height = self.target_size
            add_frame_info(red_filtered, self.video_fps, width, height, "Red Filter", 
                         self.current_frame_num, self.total_frames, timestamp)
            add_frame_info(blue_filtered, self.video_fps, width, height, "Blue/Cyan Filter",
                         self.current_frame_num, self.total_frames, timestamp)
            
            # Add tracking indicator to processed frame
            status_text = f"Tracking: {tracking_color} | Video Analysis Mode"
            cv2.putText(processed_frame, status_text, (30, 140),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
            
            # Calculate angle for tracked color and update global variable
            with angle_lock:
                if tracking_color == "red" and red_highest is not None:
                    point, _ = red_highest
                    current_angle = calculate_relative_angle(point, width)
                elif tracking_color == "blue" and blue_highest is not None:
                    point, _ = blue_highest
                    current_angle = calculate_relative_angle(point, width)
                else:
                    current_angle = None
            
            # Update display frames for main thread
            with frame_lock:
                display_frames = {
                    'raw': cv2.resize(frame, self.target_size),
                    'processed': processed_frame,
                    'red_filtered': red_filtered,
                    'blue_filtered': blue_filtered,
                    'frame_info': {
                        'current_frame': self.current_frame_num,
                        'total_frames': self.total_frames,
                        'timestamp': timestamp,
                        'angle': current_angle
                    }
                }
            
            # Control playback speed
            time.sleep(self.frame_delay)
        
        print("Video processing thread ended")

def main():
    global tracking_color, should_exit, video_paused
    
    # Video file path
    video_path = "/Users/rick/StudioProjects/FtcRoboController/test_data/matchvid.mp4"
    
    # Initialize video and calibration
    cap, orig_width, orig_height, video_fps = init_video(video_path)
    camera_matrix, dist_coeffs = get_calibration_matrices()
    target_size = (320, 240)
    
    print(f"Tracking {tracking_color} objects")
    print("Controls:")
    print("  'q' or ESC - quit")
    print("  'r' - track red objects")
    print("  'b' - track blue objects")
    print("  'SPACE' - pause/unpause video")
    print("  'l' - toggle video looping")

    # Start video processing thread
    video_thread = VideoProcessingThread(cap, camera_matrix, dist_coeffs, target_size, video_fps)
    video_thread.start()
    
    print("Main thread started - handling display and controls")
    
    try:
        while not should_exit:
            # Display frames if available
            with frame_lock:
                if display_frames:
                    # Show all streams
                    cv2.imshow('Raw Video Stream', display_frames.get('raw'))
                    cv2.imshow('Undistorted Stream', display_frames.get('processed'))
                    cv2.imshow('Red Filtered Stream', display_frames.get('red_filtered'))
                    cv2.imshow('Blue/Cyan Filtered Stream', display_frames.get('blue_filtered'))
                    
                    # Print angle information if tracking something
                    frame_info = display_frames.get('frame_info', {})
                    if frame_info.get('angle') is not None:
                        angle = frame_info['angle']
                        timestamp = frame_info['timestamp']
                        print(f"Time: {timestamp:.1f}s - {tracking_color.capitalize()} object angle: {angle:.2f}°")
            
            # Handle user input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # 27 is ESC key
                should_exit = True
                break
            elif key == ord('r'):
                tracking_color = "red"
                print("Switching to track red objects")
            elif key == ord('b'):
                tracking_color = "blue"
                print("Switching to track blue objects")
            elif key == ord(' '):  # SPACE key
                video_paused = not video_paused
                print(f"Video {'paused' if video_paused else 'resumed'}")
            
            # Small sleep to prevent excessive CPU usage
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received")
        should_exit = True
    
    # Clean up
    print("Shutting down...")
    should_exit = True
    
    # Wait for video thread to finish
    if video_thread.is_alive():
        video_thread.join(timeout=2.0)
    
    cap.release()
    cv2.destroyAllWindows()
    print("Video analysis ended")

if __name__ == "__main__":
    main()