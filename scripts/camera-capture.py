from picamera2 import Picamera2
from datetime import datetime
from PIL import Image, ImageDraw, ImageFont
import os

# Initialize the camera
picam2 = Picamera2()
picam2.configure(picam2.create_still_configuration())

# Start the camera
picam2.start()

# Create a new directory with a timestamp
base_dir = "/home/rahul/captured_frames"
timestamp_dir = datetime.now().strftime("%Y%m%d_%H%M%S")  # Format: YYYYMMDD_HHMMSS
save_dir = os.path.join(base_dir, timestamp_dir)
os.makedirs(save_dir, exist_ok=True)

print(f"Saving frames to: {save_dir}")
print("Press Ctrl+C to stop capturing.")

try:
    while True:
        # Capture an image
        image = picam2.capture_array()
        
        # Add timestamp to the image
        frame_timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")  # Timestamp for overlay
        image_pil = Image.fromarray(image)
        # draw = ImageDraw.Draw(image_pil)
        # font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 32)
        # draw.text((10, 10), frame_timestamp, font=font, fill=(255, 255, 255))

        # Save the image with its own timestamped filename
        filename = datetime.now().strftime("%Y%m%d_%H%M%S") + ".jpg"  # Format: YYYYMMDD_HHMMSS.jpg
        frame_path = os.path.join(save_dir, filename)
        image_pil.save(frame_path)

        print(f"Saved: {frame_path}")

except KeyboardInterrupt:
    # Stop capturing on Ctrl+C
    print("\nStopped capturing.")

finally:
    picam2.stop()
