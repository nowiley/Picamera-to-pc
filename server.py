#!/urs/bin/python3
"""
This is a server file that will run on raspberry pi which will provide a socket connection to clients
and will use Picamera2 to capture images and send them to the clients.
FROM https://github.com/raspberrypi/picamera2/blob/main/examples/capture_stream.py#L8
"""
import socket
import time
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput

picam2 = Picamera2()
video_config = picam2.create_video_configuration({"size": (640, 480)})
picam2.configure(video_config)
encoder = JpegEncoder()

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", 8000))
    sock.listen()

    picam2.encoders = encoder

    conn, addr = sock.accept()
    print("Connection from", addr)
    stream = conn.makefile("wb")
    encoder.output = FileOutput(stream)
    picam2.start_encoder()
    picam2.start()
    print("Picamera2 and encoder started")
    try:
        while True:  # Stream forever
            time.sleep(1)  # Prevent the loop from running too fast
    except KeyboardInterrupt:
        print("Picamera2 and encoder stopped")
        picam2.stop()
        picam2.stop_encoder()
        conn.close()

