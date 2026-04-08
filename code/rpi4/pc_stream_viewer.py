import socket
import cv2
import numpy as np
import struct
import os

"""Simple TCP stream viewer that receives JPEG frames and displays/saves them."""

# Configuration
HOST: str = '0.0.0.0'  # Listen on all interfaces
PORT: int = 9999       # Port to listen on

def main() -> None:
    """Start TCP server, receive frames, save them, and render live preview.

    The server keeps listening for new senders after a disconnect.
    The program exits only when the user presses 'q'.
    Make sure to change the Paths accordingly.
    """
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    save_dir = r"C:\code\pics-da" # make sure this is your actual directory for saving frames
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    count = 0
    print(f"Listening for connection on {HOST}:{PORT}...")
    payload_size = struct.calcsize(">L")
    should_exit = False

    try:
        while not should_exit:
            conn, addr = server_socket.accept()
            print(f"Connected by {addr}")
            data = b""

            try:
                while True:
                    # Retrieve message size
                    while len(data) < payload_size:
                        packet = conn.recv(4096)
                        if not packet:
                            raise ConnectionError("Sender disconnected")
                        data += packet

                    packed_msg_size = data[:payload_size]
                    data = data[payload_size:]
                    msg_size = struct.unpack(">L", packed_msg_size)[0]

                    # Retrieve all data based on message size
                    while len(data) < msg_size:
                        packet = conn.recv(4096)
                        if not packet:
                            raise ConnectionError("Sender disconnected")
                        data += packet

                    frame_data = data[:msg_size]
                    data = data[msg_size:]

                    # Decode frame
                    frame = cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), cv2.IMREAD_COLOR)

                    if frame is not None:
                        # Save frame to file
                        filename = os.path.join(save_dir, f'frame{count}.jpg')
                        cv2.imwrite(filename, frame)
                        count += 1

                        cv2.imshow('Remote Stream', frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            should_exit = True
                            break
            except ConnectionError:
                print("Connection lost. Waiting for a new connection...")
            finally:
                conn.close()

    except Exception as e:
        print(f"Error: {e}")
    finally:
        server_socket.close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
