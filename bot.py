import cv2
import numpy as np
from PIL import ImageGrab
import pyautogui
import time
from ultralytics import YOLO

class TapGameBot:
    def __init__(self, model_path, confidence_threshold=0.6):
        self.confidence_threshold = confidence_threshold
        # Load YOLO model
        self.model = YOLO(model_path)
        # Prevent mouse movement from triggering SafetyNet
        pyautogui.FAILSAFE = False
        # Add small pause between actions
        pyautogui.PAUSE = 0.1
        
    def capture_screen(self):
        """Capture the game screen"""
        # Get screen size
        screen_width, screen_height = pyautogui.size()
        print(f"Screen size: {screen_width}x{screen_height}")
        
        # Capture specific region - adjust these values based on your game window position
        game_region = ImageGrab.grab()
        self.offset_x = 0  # Add offset if game window isn't at (0,0)
        self.offset_y = 0
        
        screen = np.array(game_region)
        return cv2.cvtColor(screen, cv2.COLOR_RGB2BGR)

    def process_detections(self, detections):
        """Process YOLO detections to identify character and blocks"""
        character_pos = None
        blocks = []
        
        # Process results from YOLO model
        for result in detections:
            boxes = result.boxes
            print(f"Found {len(boxes)} detections")
            
            for box in boxes:
                # Get confidence and class
                confidence = float(box.conf[0])
                class_id = int(box.cls[0])
                
                print(f"Detection - Class: {class_id}, Confidence: {confidence:.2f}")
                
                if confidence < self.confidence_threshold:
                    continue
                
                # Get coordinates
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                w = x2 - x1
                h = y2 - y1
                center_x = x1 + w/2
                center_y = y1 + h/2
                
                if class_id == 0:  # Character class
                    character_pos = (center_x, center_y)
                    print(f"Character position: ({center_x:.1f}, {center_y:.1f})")
                elif class_id == 1:  # Block class
                    blocks.append({
                        'center': (center_x, center_y),
                        'coords': (x1, y1, w, h)
                    })
                    print(f"Block found at: ({center_x:.1f}, {center_y:.1f})")
        
        return character_pos, blocks

    def find_next_block(self, character_pos, blocks):
        """Find the next closest block above the character"""
        if not character_pos or not blocks:
            return None
            
        print(f"\nFinding next block...")
        print(f"Character at: {character_pos}")
        print(f"Total blocks: {len(blocks)}")
        
        # Sort blocks by vertical position (y-coordinate)
        blocks.sort(key=lambda x: x['center'][1])
        
        # Find the next block above the character
        next_block = None
        min_distance = float('inf')
        
        for block in blocks:
            block_y = block['center'][1]
            if block_y < character_pos[1]:  # Block is above character
                # Calculate Euclidean distance
                distance = ((block['center'][0] - character_pos[0])**2 + 
                          (block_y - character_pos[1])**2)**0.5
                print(f"Block at {block['center']} - Distance: {distance:.1f}")
                if distance < min_distance:
                    min_distance = distance
                    next_block = block
        
        if next_block:
            print(f"Selected block at: {next_block['center']}")
        else:
            print("No suitable block found")
                
        return next_block

    def tap_block(self, block):
        """Tap the center of the detected block"""
        if block:
            # Get screen coordinates
            screen_x = int(block['center'][0] + self.offset_x)
            screen_y = int(block['center'][1] + self.offset_y)
            
            print(f"\nTapping block at screen coordinates: ({screen_x}, {screen_y})")
            
            # Move mouse first (helps with debugging)
            pyautogui.moveTo(screen_x, screen_y, duration=0.1)
            # Click
            pyautogui.click(x=screen_x, y=screen_y)
            
            print("Click executed")
            # Small delay to prevent double-clicking
            time.sleep(0.2)

    def play_game(self):
        """Main game loop"""
        last_tap_time = 0
        MIN_TAP_INTERVAL = 0.5  # Minimum time between taps in seconds
        
        print("Starting game loop...")
        while True:
            try:
                # Capture game screen
                frame = self.capture_screen()
                
                # Run YOLO detection
                detections = self.model(frame)
                
                # Process detections
                character_pos, blocks = self.process_detections(detections)
                
                # Find next target block
                next_block = self.find_next_block(character_pos, blocks)
                
                # Check if enough time has passed since last tap
                current_time = time.time()
                if next_block and (current_time - last_tap_time) >= MIN_TAP_INTERVAL:
                    print(f"\nTime since last tap: {current_time - last_tap_time:.2f}s")
                    self.tap_block(next_block)
                    last_tap_time = current_time
                
                # Add small delay to prevent excessive CPU usage
                time.sleep(0.05)
                
            except Exception as e:
                print(f"Error occurred: {e}")
                import traceback
                print(traceback.format_exc())
                continue

# Example usage
if __name__ == "__main__":
    model_path = 'trained_model/best-simulation.pt'
    bot = TapGameBot(model_path)
    try:
        bot.play_game()
    except KeyboardInterrupt:
        print("Bot stopped by user")
