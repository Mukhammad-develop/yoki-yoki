import torch
import cv2
import numpy as np
import mss
import pyautogui
from ultralytics import YOLO

model = YOLO('yoki-yoki/trained_model/best-simulation.pt')

sct = mss.mss()
game_area = {"top": 100, "left": 100, "width": 1920, "height": 1080}  # Define the game screen area

def capture_screen():
    screenshot = sct.grab(game_area)
    img = np.array(screenshot)
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return img

# Initialize previous character position to detect movement direction
previous_character_y = None

def detect_objects(img):
    results = model(img)
    detections = results[0].boxes  # Access detections

    buttons = {'S': [], 'A': [], 'Z': [], 'X': [], 'C': []}
    blocks = []
    character = None

    for box in detections:
        xyxy = box.xyxy[0].tolist()  # Coordinates of the box
        cls = int(box.cls[0])  # Class of the detection
        y_center = (xyxy[1] + xyxy[3]) / 2  # y-coordinate center of the detection

        # Map classes to buttons or blocks
        if cls == 0:  # Assuming 0 is 'S'
            buttons['S'].append(y_center)
        elif cls == 1:  # Assuming 1 is 'A'
            buttons['A'].append(y_center)
        elif cls == 2:  # Assuming 2 is 'Z'
            buttons['Z'].append(y_center)
        elif cls == 3:  # Assuming 3 is 'X'
            buttons['X'].append(y_center)
        elif cls == 4:  # Assuming 4 is 'C'
            buttons['C'].append(y_center)
        elif cls == 5:  # Assuming 5 is 'block'
            blocks.append(y_center)
        elif cls == 6:  # Assuming 6 is 'character'
            character = y_center

    return buttons, blocks, character

def find_nearest_block_below(character_y, blocks):
    nearest_block_y = None
    min_distance = float('inf')

    # Find the closest block below the character
    for block_y in blocks:
        if block_y > character_y:  # Only consider blocks below the character
            distance = block_y - character_y
            if distance < min_distance:
                min_distance = distance
                nearest_block_y = block_y

    return nearest_block_y

def find_nearest_button(buttons, block_y):
    nearest_button = None
    min_distance = float('inf')
    
    for letter, y_coords in buttons.items():
        for y in y_coords:
            distance = abs(block_y - y)
            if distance < min_distance:
                min_distance = distance
                nearest_button = (letter, y)

    return nearest_button

def play_game(screen):
    global previous_character_y
    
    # Detect objects in the current frame
    buttons, blocks, character_y = detect_objects(screen)

    # Check if we have a valid character position
    if character_y is not None and previous_character_y is not None:
        # Determine if the character is moving down
        is_moving_down = character_y > previous_character_y

        if is_moving_down:
            # Find the nearest block below the character
            nearest_block_y = find_nearest_block_below(character_y, blocks)

            if nearest_block_y is not None:
                # Find the nearest button associated with the detected block
                nearest_button = find_nearest_button(buttons, nearest_block_y)

                if nearest_button:
                    letter, button_y = nearest_button
                    print(f"Clicking on letter: {letter} at y: {button_y}")

                    # Define x-coordinates for each letter button
                    letter_x_coords = {
                        'S': 56,  # Replace with actual x-coordinate for 'S'
                        'A': 190,  # Replace with actual x-coordinate for 'A'
                        'Z': 294,  # Replace with actual x-coordinate for 'Z'
                        'X': 416,  # Replace with actual x-coordinate for 'X'
                        'C': 538   # Replace with actual x-coordinate for 'C'
                    }

                    # Simulate a click on the nearest button
                    pyautogui.click(letter_x_coords[letter], button_y)

    # Update the previous character y-coordinate for the next frame
    previous_character_y = character_y

        
while True:
    screen = capture_screen()
    buttons, blocks, character = detect_objects(screen)
    play_game(screen)