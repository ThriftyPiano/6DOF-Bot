#!/usr/bin/env python3
import cv2
import os

def extract_frame_at_time(video_path, time_seconds, output_path):
    """Extract and save a frame at a specific time from a video."""
    # Open the video
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Could not open video file: {video_path}")
        return False
    
    # Get video properties
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    duration = total_frames / fps
    
    print(f"Video FPS: {fps}")
    print(f"Video duration: {duration:.1f} seconds")
    print(f"Extracting frame at {time_seconds} seconds...")
    
    # Calculate the frame number for the desired time
    target_frame = int(time_seconds * fps)
    
    if target_frame >= total_frames:
        print(f"Error: Requested time {time_seconds}s is beyond video duration {duration:.1f}s")
        cap.release()
        return False
    
    # Set the video position to the target frame
    cap.set(cv2.CAP_PROP_POS_FRAMES, target_frame)
    
    # Read the frame
    ret, frame = cap.read()
    if not ret:
        print(f"Error: Could not read frame at {time_seconds} seconds")
        cap.release()
        return False
    
    # Save the frame
    success = cv2.imwrite(output_path, frame)
    if success:
        print(f"Frame saved successfully to: {output_path}")
        print(f"Frame dimensions: {frame.shape[1]}x{frame.shape[0]}")
    else:
        print(f"Error: Could not save frame to {output_path}")
    
    cap.release()
    return success

if __name__ == "__main__":
    # Video file path
    video_path = "/Users/rick/StudioProjects/FtcRoboController/test_data/matchvid.mp4"
    
    # Time to extract (11 seconds)
    time_seconds = 11.0
    
    # Output file path
    output_path = "/Users/rick/StudioProjects/FtcRoboController/test_data/frame_at_11s.jpg"
    
    # Extract the frame
    extract_frame_at_time(video_path, time_seconds, output_path)