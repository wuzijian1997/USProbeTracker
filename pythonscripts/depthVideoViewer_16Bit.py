import imageio.v3 as iio
import numpy as np
import cv2

# Path to the 16-bit depth video
depth_video_path = "../data/P0/depthvideo_27-02-2025_19-32-11.mkv"

# Open the video using imageio (FFmpeg backend)
reader = iio.imiter(depth_video_path, plugin="pyav", format="gray16le")

frame_idx = 0  # Track frame index

for depth_frame in reader:
    # Convert to numpy array and ensure it's 16-bit
    depth_frame = np.array(depth_frame, dtype=np.uint16)

    # Check for frame skipping (if frame values change drastically)
    print(f"Frame {frame_idx} dtype: {depth_frame.dtype}, Min: {depth_frame.min()}, Max: {depth_frame.max()}")
    
    # Normalize for better visualization
    depth_clipped = np.clip(depth_frame, 0, 4095)

    # Convert to 8-bit grayscale for display
    depth_display_8bit = ((depth_clipped / 4095)*255).astype(np.uint8)

    # Show the depth frame
    cv2.imshow("16-bit Depth Video", depth_display_8bit)

    # Step through frames manually
    key = cv2.waitKey(0) & 0xFF
    if key == 27:  # ESC key to exit
        break

    frame_idx += 1  # Increment frame index

cv2.destroyAllWindows()
