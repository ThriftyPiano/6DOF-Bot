#!/usr/bin/env python3
import cv2
import numpy as np
import sys
from typing import Tuple, List, Optional

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

def detect_color_contours_full_frame(frame: np.ndarray, color: str) -> Tuple[List[np.ndarray], np.ndarray]:
    """Detect contours for specified color in the full frame (no region masking)."""
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
    elif color == "blue":
        lower_blue = (105, 140, 70)
        upper_blue = (115, 255, 225)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
    elif color == "yellow":
        lower_yellow = (15, 50, 50)
        upper_yellow = (50, 255, 255)
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
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

def find_top_straight_line(contour: np.ndarray, tolerance: float = 5.0) -> Optional[Tuple[Tuple[int, int], Tuple[int, int], Tuple[int, int]]]:
    """Find the top straight line of a contour and return its endpoints and midpoint.
    
    Args:
        contour: The contour to analyze
        tolerance: Y-coordinate tolerance for considering points on the same horizontal line
    
    Returns:
        Tuple of (left_point, right_point, midpoint) if found, None otherwise
    """
    if contour is None or len(contour) < 3:
        return None
        
    contour_points = contour.squeeze()
    if len(contour_points.shape) != 2 or contour_points.shape[1] != 2:
        return None
    
    # Find the minimum Y coordinate (topmost point)
    min_y = np.min(contour_points[:, 1])
    
    # Find all points within tolerance of the minimum Y
    top_points = contour_points[np.abs(contour_points[:, 1] - min_y) <= tolerance]
    
    if len(top_points) < 2:
        return None
    
    # Sort by X coordinate to find leftmost and rightmost
    top_points = top_points[np.argsort(top_points[:, 0])]
    
    left_point = tuple(top_points[0].astype(int))
    right_point = tuple(top_points[-1].astype(int))
    
    # Calculate midpoint
    midpoint = (
        int((left_point[0] + right_point[0]) / 2),
        int((left_point[1] + right_point[1]) / 2)
    )
    
    return left_point, right_point, midpoint

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

def filter_topmost_yellow_contour(contours: List[np.ndarray]) -> List[np.ndarray]:
    """Filter yellow contours to keep only the topmost one."""
    if not contours:
        return contours
    
    # Find the contour with the smallest (topmost) Y coordinate
    topmost_contour = None
    topmost_y = float('inf')
    
    for contour in contours:
        if cv2.contourArea(contour) >= 50:  # Minimum area threshold
            result = find_highest_point(contour)
            if result is not None:
                point, y = result
                if y < topmost_y:
                    topmost_y = y
                    topmost_contour = contour
    
    return [topmost_contour] if topmost_contour is not None else []

def draw_results(frame: np.ndarray, contours: List[np.ndarray], 
                highest_point: Optional[Tuple[tuple, float]], 
                color: Tuple[int, int, int], draw_top_line: bool = False) -> None:
    """Draw contours and highest point on the frame.
    
    Args:
        frame: The frame to draw on
        contours: List of contours to draw
        highest_point: Highest point to mark
        color: Color for drawing contours
        draw_top_line: Whether to find and draw top straight line (for yellow contours)
    """
    if contours:
        cv2.drawContours(frame, contours, -1, color, 2)
        # Only draw highest point if not drawing top line (skip for yellow contours)
        if highest_point is not None and not draw_top_line:
            point, y = highest_point
            cv2.circle(frame, point, 4, (0, 255, 0), -1)
        
        # Draw top straight line for yellow contours
        if draw_top_line:
            for contour in contours:
                if cv2.contourArea(contour) >= 50:  # Only for reasonably sized contours
                    line_result = find_top_straight_line(contour)
                    if line_result is not None:
                        left_point, right_point, midpoint = line_result
                        
                        # Draw the top line in bright green
                        cv2.line(frame, left_point, right_point, (0, 255, 0), 3)
                        
                        # Draw the midpoint as a large red circle
                        cv2.circle(frame, midpoint, 8, (0, 0, 255), -1)
                        
                        # Add text label for the midpoint
                        cv2.putText(frame, f"Mid: ({midpoint[0]}, {midpoint[1]})", 
                                   (midpoint[0] - 60, midpoint[1] - 15),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

def detect_yellow_in_contour_interiors(frame: np.ndarray, red_contours: List[np.ndarray], blue_contours: List[np.ndarray]) -> Tuple[List[np.ndarray], np.ndarray]:
    """Detect yellow contours only within the interior regions of red and blue contours."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    height, width = frame.shape[:2]
    
    # Create mask for yellow detection
    lower_yellow = (15, 50, 50)
    upper_yellow = (50, 255, 255)
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    # Create interior mask from red and blue contours
    interior_mask = np.zeros((height, width), dtype=np.uint8)
    
    # Add interior regions of red contours with erosion to ensure true interior
    for contour in red_contours:
        if cv2.contourArea(contour) >= 100:  # Use larger contours only
            temp_mask = np.zeros((height, width), dtype=np.uint8)
            cv2.fillPoly(temp_mask, [contour], 255)
            
            # Check if contour touches frame edges (for U-shaped contours)
            x, y, w, h = cv2.boundingRect(contour)
            touches_edge = (x <= 3 or x + w >= width - 3 or y <= 3 or y + h >= height - 3)
            
            if touches_edge:
                # For edge-touching contours (like U-shapes), use minimal erosion
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
                temp_mask = cv2.erode(temp_mask, kernel, iterations=1)
            else:
                # For interior contours, use lighter erosion to reduce offset
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                temp_mask = cv2.erode(temp_mask, kernel, iterations=1)
            
            interior_mask = cv2.bitwise_or(interior_mask, temp_mask)
    
    # Add interior regions of blue contours with erosion to ensure true interior
    for contour in blue_contours:
        if cv2.contourArea(contour) >= 100:  # Use larger contours only
            temp_mask = np.zeros((height, width), dtype=np.uint8)
            cv2.fillPoly(temp_mask, [contour], 255)
            
            # Check if contour touches frame edges (for U-shaped contours)
            x, y, w, h = cv2.boundingRect(contour)
            touches_edge = (x <= 3 or x + w >= width - 3 or y <= 3 or y + h >= height - 3)
            
            if touches_edge:
                # For edge-touching contours (like U-shapes), use minimal erosion
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
                temp_mask = cv2.erode(temp_mask, kernel, iterations=1)
            else:
                # For interior contours, use lighter erosion to reduce offset
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                temp_mask = cv2.erode(temp_mask, kernel, iterations=1)
            
            interior_mask = cv2.bitwise_or(interior_mask, temp_mask)
    
    # Apply interior mask to yellow detection
    yellow_mask = cv2.bitwise_and(yellow_mask, interior_mask)
    
    # Fill gaps caused by text within yellow areas
    # Use closing to fill small holes and gaps within yellow regions
    fill_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8, 8))
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, fill_kernel, iterations=2)
    
    # Clean up with opening to remove small noise while preserving filled areas
    clean_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, clean_kernel)
    
    # Find contours in the masked yellow regions
    contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filtered_frame = cv2.bitwise_and(frame, frame, mask=yellow_mask)
    
    return contours, filtered_frame

def add_image_info(frame: np.ndarray, width: int, height: int, label: str) -> None:
    """Add resolution and label information to the frame."""
    cv2.putText(frame, f"Resolution: {width}x{height}", (30, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
    if label:
        cv2.putText(frame, label, (30, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

def main():
    # Image file path
    image_path = "/Users/rick/StudioProjects/FtcRoboController/python/goalImages/blurryashec.jpg"
    
    # Load the image
    image = cv2.imread(image_path)
    if image is None:
        print(f"Error: Could not load image file: {image_path}")
        sys.exit(1)
    
    orig_height, orig_width = image.shape[:2]
    print(f"Original image resolution: {orig_width}x{orig_height}")
    
    # Initialize calibration
    camera_matrix, dist_coeffs = get_calibration_matrices()
    target_size = (800, 600)  # Larger size for single image display
    
    # Undistort and scale image
    processed_image = undistort_and_scale(image, camera_matrix, dist_coeffs, target_size)
    width, height = target_size
    
    # First, get full-frame red and blue contours for yellow detection
    red_contours_full, _ = detect_color_contours_full_frame(processed_image, "red")
    blue_contours_full, _ = detect_color_contours_full_frame(processed_image, "blue")
    
    # Process yellow contours within full-frame red/blue interiors
    yellow_contours, yellow_filtered = detect_yellow_in_contour_interiors(processed_image, red_contours_full, blue_contours_full)
    # Filter to keep only the topmost yellow contour
    filtered_yellow_contours = filter_topmost_yellow_contour(yellow_contours)
    valid_yellow_contours, yellow_highest = process_contours(filtered_yellow_contours, 0)  # No minimum area for yellow
    
    # Process red contours for display
    red_contours, red_filtered = detect_color_contours_full_frame(processed_image, "red")
    valid_red_contours, red_highest = process_contours(red_contours, 400)  # Minimum area 400
    
    # Process blue contours for display
    blue_contours, blue_filtered = detect_color_contours_full_frame(processed_image, "blue")
    valid_blue_contours, blue_highest = process_contours(blue_contours, 400)  # Minimum area 400
    
    # Create display frames
    raw_display = cv2.resize(image, target_size)
    processed_display = processed_image.copy()
    red_display = red_filtered.copy()
    blue_display = blue_filtered.copy()
    yellow_display = yellow_filtered.copy()
    
    # Draw results on each frame
    draw_results(yellow_display, valid_yellow_contours, yellow_highest, (0, 255, 255), draw_top_line=True)
    draw_results(red_display, valid_red_contours, red_highest, (0, 0, 255))
    draw_results(blue_display, valid_blue_contours, blue_highest, (255, 0, 0))
    
    # Overlay yellow contours onto red and blue filtered streams
    draw_results(red_display, valid_yellow_contours, yellow_highest, (0, 255, 255), draw_top_line=True)
    draw_results(blue_display, valid_yellow_contours, yellow_highest, (0, 255, 255), draw_top_line=True)
    
    # Add image information
    add_image_info(raw_display, orig_width, orig_height, "Raw Image")
    add_image_info(processed_display, width, height, "Undistorted Image")
    add_image_info(red_display, width, height, "Red Filtered + Yellow")
    add_image_info(blue_display, width, height, "Blue Filtered + Yellow")
    add_image_info(yellow_display, width, height, "Yellow Filtered")
    
    print("Controls:")
    print("  'q' or ESC - quit")
    print("  Any other key - refresh display")
    
    # Display all images
    while True:
        cv2.imshow('Raw Image', raw_display)
        cv2.imshow('Undistorted Image', processed_display)
        cv2.imshow('Red Filtered Stream', red_display)
        cv2.imshow('Blue Filtered Stream', blue_display)
        cv2.imshow('Yellow Filtered Stream', yellow_display)
        
        # Print analysis results
        if valid_yellow_contours:
            print(f"Found {len(valid_yellow_contours)} yellow contour(s)")
            for i, contour in enumerate(valid_yellow_contours):
                line_result = find_top_straight_line(contour)
                if line_result is not None:
                    left_point, right_point, midpoint = line_result
                    print(f"Yellow contour {i+1}: Top line from {left_point} to {right_point}, midpoint at {midpoint}")
        
        # Wait for key press
        key = cv2.waitKey(0) & 0xFF
        if key == ord('q') or key == 27:  # 27 is ESC key
            break
    
    # Clean up
    cv2.destroyAllWindows()
    print("Image analysis completed")

if __name__ == "__main__":
    main()