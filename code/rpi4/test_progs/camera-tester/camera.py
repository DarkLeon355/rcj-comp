from picamera2 import Picamera2
from time import sleep

# Initialize camera
picam2 = Picamera2()
camera_config = picam2.create_video_configuration(main={"size": (2592, 1944)})
picam2.configure(camera_config)
picam2.start()

# Wait for camera to initialize
sleep(2)

# Capture image
picam2.capture_file("/home/da-pi/camera-tester/image.jpg")