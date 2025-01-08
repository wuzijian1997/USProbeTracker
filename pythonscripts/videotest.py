import cv2
import numpy as np
import os

def depth_to_heatmap(depth_frame, scale_factor=0.001):
    """
    Convert a depth frame to a heat map.

    Args:
        depth_frame (numpy.ndarray): 16-bit depth frame.
        scale_factor (float): Conversion factor from depth values to meters.

    Returns:
        numpy.ndarray: Heat map image.
    """
    # Convert depth values to meters
    depth_in_meters = depth_frame * scale_factor

    # Normalize depth values for visualization (0 to 1 range)
    normalized_depth = cv2.normalize(depth_in_meters, None, 0, 1, cv2.NORM_MINMAX)

    # Convert normalized depth to 8-bit
    depth_8bit = (normalized_depth * 255).astype(np.uint8)

    # Apply a color map to the depth image
    heatmap = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)

    return heatmap

def process_depth_frames(folder_path, scale_factor=0.001):
    """
    Process depth frames in a folder and display as heat maps.

    Args:
        folder_path (str): Path to the folder containing depth .png images.
        scale_factor (float): Conversion factor from depth values to meters.
    """
    # Get list of PNG files in the folder
    png_files = [f for f in os.listdir(folder_path) if f.endswith('.png')]
    
    if not png_files:
        print("No PNG files found in the specified folder.")
        return

    for file_name in sorted(png_files):
        # Load the depth frame as a 16-bit image
        file_path = os.path.join(folder_path, file_name)
        depth_frame = cv2.imread(file_path, cv2.IMREAD_UNCHANGED)

        if depth_frame is None or depth_frame.dtype != np.uint16:
            print(f"Skipping {file_name}: Not a valid 16-bit depth image.")
            continue

        # Convert the depth frame to a heat map
        heatmap = depth_to_heatmap(depth_frame, scale_factor)

        # Display the heat map
        cv2.imshow("Depth Heatmap", heatmap)
        print(f"Displaying {file_name}")

        # Wait for a key press (press any key to proceed to the next frame, or 'q' to quit)
        if cv2.waitKey(0) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Path to the folder containing depth .png images
    folder_path = "../data/P0/depthvideo_07-01-2025_13-48-48.mkv"

    # Conversion factor from depth values to meters (adjust based on your RealSense configuration)
    scale_factor = 0.001  # Example: 1 depth unit = 1 mm

    process_depth_frames(folder_path, scale_factor)