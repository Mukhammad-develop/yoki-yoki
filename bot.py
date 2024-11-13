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
    detections = results[0].boxes  # Access the detections directly

    blocks = []
    character = None
    for box in detections:
        xyxy = box.xyxy[0].tolist()  # Extract coordinates
        cls = int(box.cls[0])  # Extract class
        if cls == 0:  # Assuming 0 is block class
            blocks.append(xyxy)
        elif cls == 1:  # Assuming 1 is character class
            character = xyxy
    return blocks, character

def play_game(blocks, character):
    if character:
        character_x = (character[0] + character[2]) / 2
        character_y = (character[1] + character[3]) / 2

        # Find the closest block above the character
        closest_block = None
        min_distance = float('inf')
        for block in blocks:
            block_x = (block[0] + block[2]) / 2
            block_y = (block[1] + block[3]) / 2
            distance = character_y - block_y
            if 0 < distance < min_distance:  # Look for blocks above within a minimum distance
                min_distance = distance
                closest_block = block

        if closest_block:
            block_x = (closest_block[0] + closest_block[2]) / 2
            
            # Smoothly move the cursor to hover over the target x position of the closest block
            pyautogui.moveTo(block_x, character_y)  # Adjust duration for speed

            # Short delay to allow the character to jump when it reaches the block


while True:
    screen = capture_screen()
    blocks, character = detect_objects(screen)
    play_game(blocks, character)