import torch
import cv2
import numpy as np
import mss
import pyautogui
from ultralytics import YOLO

model = YOLO('trained_model/best-simulation.pt')

sct = mss.mss()
game_area = {"top": 100, "left": 100, "width": 1920, "height": 1080}  # Define the game screen area

def capture_screen():
    screenshot = sct.grab(game_area)
    img = np.array(screenshot)
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return img

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

def find_nearest_button(buttons, character_y):
    nearest_button = None
    min_distance = float('inf')
    
    for letter, y_coords in buttons.items():
        for y in y_coords:
            distance = abs(character_y - y)
            if distance < min_distance:
                min_distance = distance
                nearest_button = (letter, y)

    return nearest_button

def play_game(screen):
    buttons, blocks, character_y = detect_objects(screen)

    if character_y is not None:
        nearest_button = find_nearest_button(buttons, character_y)
        
        if nearest_button:
            letter, y_coord = nearest_button
            print(f"Clicking on letter: {letter} at y: {y_coord}")
            
            # Define x-coordinates for each letter button (this depends on screen location of each letter)
            letter_x_coords = {
                'S': 100,  # Replace with actual x-coordinate for 'S'
                'A': 200,  # Replace with actual x-coordinate for 'A'
                'Z': 300,  # Replace with actual x-coordinate for 'Z'
                'X': 400,  # Replace with actual x-coordinate for 'X'
                'C': 500   # Replace with actual x-coordinate for 'C'
            }

            # Check if the x-coordinate is defined in letter_x_coords
            if letter in letter_x_coords:
                # Click the nearest button's x and y coordinates
                pyautogui.click(letter_x_coords[letter], y_coord)
                time.sleep(0.1)  # Add a small delay to avoid rapid clicking
            else:
                print(f"Error: X-coordinate for button '{letter}' not defined.")
        else:
            print("No button detected near the character.")
    else:
        print("Character not detected.")
        
while True:
    screen = capture_screen()
    blocks, character = detect_objects(screen)
    play_game(blocks, character)