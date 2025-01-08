
import cv2
import numpy as np

def display_depth_video_as_heatmap(video_file):
    # Open the video file
    cap = cv2.VideoCapture(video_file)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open video file: {video_file}")

    while cap.isOpened():
        # Read a frame
        ret, frame = cap.read()
        if not ret:
            break

        # Convert the frame to 16-bit if needed (depends on how it was saved)
        if frame.dtype != np.uint16:
            frame = frame.astype(np.uint16)

        # Normalize the frame for visualization
        # Scale depth values to 0-255 for heatmap
        normalized_frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX)

        # Convert to 8-bit for heatmap generation
        heatmap_frame = np.uint8(normalized_frame)

        # Apply a colormap to the normalized frame
        heatmap = cv2.applyColorMap(heatmap_frame, cv2.COLORMAP_JET)

        # Display the heatmap
        cv2.imshow("Depth Heatmap", heatmap)

        # Break on key press
        if cv2.waitKey(30) & 0xFF == ord('q'):  # Press 'q' to quit
            break

    # Release the video capture and close windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Path to the folder containing depth .png images
    video_file  = "../data/P0/depthvideo_08-01-2025_07-33-13.mp4"

    display_depth_video_as_heatmap(video_file)