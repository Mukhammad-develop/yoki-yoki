import pyautogui
import numpy as np
from ultralytics import YOLO
import cv2
import time

# Load YOLO model for object detection
model = YOLO('trained_model/best-simulation.pt')

# Define screen capture function
def capture_screen():
    screenshot = pyautogui.screenshot()
    frame = np.array(screenshot)
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return frame

# Logic to control the character
def play_game():
    while True:
        # Capture the screen and run object detection
        frame = capture_screen()
        results = model(frame)

        # Extract the positions of the character and blocks
        character = None
        blocks = []

        for result in results:
            if result.boxes is not None:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])  # Extract bounding box coordinates
                    center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                    label = int(box.cls[0])  # Get the class ID

                    if label == 0:  # Assuming '0' is the class ID for the character
                        character = (center_x, center_y)
                    elif label == 1:  # Assuming '1' is the class ID for the blocks
                        blocks.append((center_x, center_y))

        # Ensure both character and blocks are detected
        if character and blocks:
            # Find the nearest block above when moving downward
            character_x, character_y = character
            above_blocks = [block for block in blocks if block[1] < character_y]
            nearest_block = None
            min_distance = float('inf')

            for block in above_blocks:
                block_x, block_y = block
                distance = np.sqrt((character_x - block_x) ** 2 + (character_y - block_y) ** 2)
                if distance < min_distance:
                    min_distance = distance
                    nearest_block = block

            # Logic for tapping on the nearest block
            if nearest_block:
                block_x, block_y = nearest_block
                pyautogui.click(block_x, block_y)  # Simulate tap on the nearest block's coordinates

        time.sleep(0.1)  # Adjust as needed to control the game pace

# Run the game-playing function
play_game()