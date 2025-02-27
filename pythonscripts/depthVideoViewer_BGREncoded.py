
import numpy as np
import cv2

# Path to the 16-bit depth video
depth_video_path = "../data/P0/depthvideo_27-02-2025_18-50-19.avi"

# Open the encoded 12-bit depth video
cap = cv2.VideoCapture(depth_video_path)

if not cap.isOpened():
    print("Error: Could not open video file.")
    exit()

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break  # Stop if end of video

    # Extract Red and Green channels
    blue_channel = frame[:, :, 0]  # MSB (8-bit)
    green_channel = frame[:, :, 1]  # LSB (4-bit shifted left)

    # Reconstruct the 12-bit depth image
    depth_image = (blue_channel.astype(np.uint16) << 4) | (green_channel.astype(np.uint16) >> 4)

    # Normalize for display (Optional: Scale 0-4095 → 0-255)
    depth_display = cv2.convertScaleAbs(depth_image, alpha=255.0 / 4095.0)

    # Apply colormap for better visualization
    depth_colormap = cv2.applyColorMap(depth_display, cv2.COLORMAP_INFERNO)

    # Show the reconstructed depth image
    cv2.imshow("12-bit Reconstructed Depth", depth_colormap)

    # Press 'ESC' to exit
    if cv2.waitKey(30) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
