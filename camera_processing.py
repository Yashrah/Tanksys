import cv2
import numpy as np
import threading
import logging
import time

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Global variables for shared data
captured_frame = None
mask_frame = None
center_of_mass = None
lock = threading.Lock()
running = True

# Provided RGB range for filtering (static as per your requirement)
lower_rgb = np.array([100, 50, 20], dtype=np.uint8)
upper_rgb = np.array([150, 110, 90], dtype=np.uint8)

# Debugging variables
save_images = True
image_counter = 0
max_images = 5  # Maximum number of images to store
last_saved_time = 0  # Timestamp of the last saved image

def capture_and_process():
    global captured_frame, mask_frame, center_of_mass, running, image_counter, last_saved_time

    # Initialize the camera (assuming the first camera)
    try:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            logging.error("Failed to open camera.")
            return
    except Exception as e:
        logging.error(f"Camera initialization error: {e}")
        return

    # Minimum contour area to be considered as the box
    min_contour_area = 500  # Adjust this value based on your object's size in the frame

    while running:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            logging.warning("Failed to capture image from camera.")
            continue

        # Convert frame to RGB (OpenCV uses BGR by default)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Create a mask to filter the specified range
        mask = cv2.inRange(frame_rgb, lower_rgb, upper_rgb)

        # Morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask_cleaned = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask_cleaned = cv2.morphologyEx(mask_cleaned, cv2.MORPH_OPEN, kernel)

        # Find contours in the cleaned mask
        contours, _ = cv2.findContours(mask_cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter out small contours
        large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_contour_area]

        with lock:
            if large_contours:
                # Find the largest contour and calculate its center of mass
                largest_contour = max(large_contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    center_of_mass = (cx, cy)

                    # Draw a filled red circle at the center of mass on the original frame
                    cv2.circle(frame, (cx, cy), 10, (0, 0, 255), cv2.FILLED)

            captured_frame = frame
            mask_frame = mask_cleaned

            # Save images every 2 seconds
            current_time = time.time()
            if save_images and current_time - last_saved_time >= 2:
                image_name = f'debug_frame_{image_counter % max_images}.png'
                mask_name = f'debug_mask_{image_counter % max_images}.png'
                cv2.imwrite(image_name, frame)
                cv2.imwrite(mask_name, mask_cleaned)
                image_counter += 1
                last_saved_time = current_time

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Clean up resources
    cap.release()
    cv2.destroyAllWindows()

def start_camera_thread():
    threading.Thread(target=capture_and_process, daemon=True).start()

def stop_camera_thread():
    global running
    running = False

def get_center_of_mass():
    with lock:
        return center_of_mass

if __name__ == "__main__":
    start_camera_thread()
    try:
        while running:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_camera_thread()
