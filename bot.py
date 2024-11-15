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
            for obj in result.xyxy:
                label = int(obj[5])
                x1, y1, x2, y2 = map(int, obj[:4])
                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2

                if label == "character":  # Replace with the label for your character in the YOLO model
                    character = (center_x, center_y)
                elif label == "block":  # Replace with the label for blocks
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
