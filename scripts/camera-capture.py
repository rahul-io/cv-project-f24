from picamera2 import Picamera2
from datetime import datetime
from PIL import Image, ImageDraw, ImageFont
import argparse
import os

# Initialize the camera
picam2 = Picamera2()
picam2.configure(picam2.create_still_configuration())

def main():
    parser = argparse.ArgumentParser(description="Capture images from the PiCamera.")

    parser.add_argument("-c", "--continuous", action='store_true', help="capture continuous images")
    parser.add_argument("-s", "--single", action='store_true', help="capture single image")

    args = parser.parse_args()

    # Start the camera
    picam2.start()

    if args.continuous:
        capture_continuous()
    else:
        capture(os.getcwd())
        print(f"Saved frame to {os.getcwd()}")
        picam2.stop()


def capture(save_dir):
    # Capture an image
    image = picam2.capture_array()
    image_pil = Image.fromarray(image)

    # Save the image with its own timestamped filename
    filename = datetime.now().strftime("%Y%m%d_%H%M%S") + ".jpg"  # Format: YYYYMMDD_HHMMSS.jpg
    frame_path = os.path.join(save_dir, filename)
    image_pil.save(frame_path)


def capture_continuous():    
    # Create a new directory with a timestamp
    timestamp_dir = datetime.now().strftime("%Y%m%d_%H%M%S")  # Format: YYYYMMDD_HHMMSS
    save_dir = os.path.join(os.getcwd(), timestamp_dir)
    os.makedirs(save_dir, exist_ok=True)

    print(f"Saving frames to: {save_dir}")
    print("Press Ctrl+C to stop capturing.")

    try:
        while True:
            # Capture an image
            capture(save_dir)
            inp = input("")

    except KeyboardInterrupt:
        # Stop capturing on Ctrl+C
        print("\nStopped capturing.")

    finally:
        picam2.stop()

if __name__ == "__main__":
    main()