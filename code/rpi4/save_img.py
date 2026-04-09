import cv2
import os
import queue
import threading
import struct
import socket
from typing import Any, Optional

class save_img_and_stream:
    """Save processed frames locally or stream them to a remote viewer."""

    def __init__(self, **kwargs: Any) -> None:
        """Initialize output paths, optional streaming, and background save worker.

        Args:
            **kwargs: Optional configuration keys.
                stream: Enable TCP streaming when True.
                local: Enable local saving when True.
                IP: Target host for streaming.
                target_width: Output width for downscaled images.
                target_height: Output height for downscaled images.
        """

        #─────────────────────────────────────────────────────────────
        # overgiven parameters:
        #─────────────────────────────────────────────────────────────
        self.server_port = 9999
        self.client_socket: Optional[socket.socket] = None

        # Target resolution for streaming/saving (300x200)
        # This significantly reduces bandwidth and storage
        self.target_width = kwargs.get('target_width', 300)
        self.target_height = kwargs.get('target_height', 200)

        # Determine if we should stream or save locally
        # Only stream if explicitly enabled with stream=True
        stream_enabled = kwargs.get("stream", False)
        self.local_save_enabled = kwargs.get("local", False)
        
        if stream_enabled:
            # Stream is enabled, check if we have an IP
            if kwargs.get('IP', None) is not None:
                self.server_host = kwargs.get('IP')
            else:
                self.server_host = "10.70.66.186"  # default to a default hostname
            self.streaming = True
        else:
            # Stream not enabled, save locally
            self.streaming = False
        #─────────────────────────────────────────────────────────────

        #─────────────────────────────────────────────────────────────
        # DEFINITIONS:
        #─────────────────────────────────────────────────────────────

        # Get the directory where this file is located
        base_dir = os.path.dirname(os.path.abspath(__file__))
        self.path_img_lines = os.path.join(base_dir, "edited")
        self.path_img = os.path.join(base_dir, "original")
        self.path_bin = os.path.join(base_dir, "binary")

        #directions file path
        self.path_dir = os.path.join(base_dir, "directions.txt")
        # junction centers file path
        self.junctions_centers_file_dir = os.path.join(base_dir, "junctions_centers.txt")
        # green dots file path
        self.green_dots_file_dir = os.path.join(base_dir, "green_dots.txt")

        #edited images paths
        self.path_edited_line = os.path.join(base_dir, "edited/line")
        self.path_edited_junction = os.path.join(base_dir, "edited/junction")
        #binary images paths
        self.path_binary_line = os.path.join(base_dir, "binary/line")
        self.path_binary_junction = os.path.join(base_dir, "binary/junction")
        #original images paths
        self.path_original_line = os.path.join(base_dir, "original/line")
        self.path_original_junction = os.path.join(base_dir, "original/junction")
        #green mask images paths
        self.path_green_line = os.path.join(base_dir, "green_mask/line")
        self.path_green_junction = os.path.join(base_dir, "green_mask/junction")
        #─────────────────────────────────────────────────────────────


        #─────────────────────────────────────────────────────────────
        # SETUP:
        #─────────────────────────────────────────────────────────────
        if self.local_save_enabled:
            self.open_files()
        if self.streaming:
            self.connect_socket()

        # Initialize queue and start worker thread for background saving
        self.queue = queue.Queue(maxsize=120)
        self.running = True
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()
        #─────────────────────────────────────────────────────────────

    def connect_socket(self) -> None:
        """Connect to the remote stream receiver over TCP."""
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.server_host, self.server_port))
            print(f"Connected to stream server at {self.server_host}:{self.server_port}")
        except:
            raise Exception(f"Could not connect to stream server at {self.server_host}:{self.server_port}")

    def open_files(self) -> None:
        """Create fresh metadata text files used for local debug output."""
        # Check if the files exist and delete them
        if os.path.exists(self.path_dir):
            try:
                os.remove(self.path_dir)
                print(f"Deleted existing file: {self.path_dir}")
            except:
                raise Exception("Error in image saving, could not delete the existing directions file")
    
        if os.path.exists(self.junctions_centers_file_dir):
            try:
                os.remove(self.junctions_centers_file_dir)
                print(f"Deleted existing file: {self.junctions_centers_file_dir}")
            except Exception:
                raise Exception("Error in image saving, could not delete the existing junctions centers file")
                    

        if os.path.exists(self.green_dots_file_dir):
            try:
                os.remove(self.green_dots_file_dir)
                print(f"Deleted existing file: {self.green_dots_file_dir}")
            except Exception:
                raise Exception("Error in image saving, could not delete the existing green dots file")

        # Create a new files
        try:
            self.points = open(self.path_dir, "a")
        except:
            raise Exception("Error in image saving, could not create the directions file")
        
        try:
            self.junctions_centers_file = open(self.junctions_centers_file_dir, "a")
        except Exception:
            raise Exception("Error in image saving, could not create the junctions centers file")
        
        try:
            self.green_dots = open(self.green_dots_file_dir, "a")
        except Exception:
            raise Exception("Error in image saving, could not create the green dots file")

    def save(self, **kwargs: Any) -> None:
        """Queue one frame packet for asynchronous save/stream processing.

        Args:
            **kwargs: Frame payload with the following required keys:
                edited, original, binary, green,
                cx_list, cy_list, junctionflag,
                junctioncenter, greendots, count.
        """
        required_keys = (
            "edited",
            "original",
            "binary",
            "green",
            "cx_list",
            "cy_list",
            "junctionflag",
            "junctioncenter",
            "greendots",
            "count",
        )
        missing = [key for key in required_keys if key not in kwargs]
        if missing:
            print(f"Warning: Missing save() kwargs: {missing}. Dropping frame.")
            return

        payload = {key: kwargs[key] for key in required_keys}
        try:
            self.queue.put_nowait(payload)
        except queue.Full:
            print("Warning: Save queue full, dropping frame")

    def _worker(self) -> None:
        """Background worker: consume queued frames and save or stream them."""
        while self.running:
            try:
                # Wait for data
                payload = self.queue.get(timeout=1)
            except queue.Empty:
                continue
            
            # Unpack arguments
            edited = payload["edited"]
            original = payload["original"]
            binary = payload["binary"]
            green = payload["green"]
            cx_list = payload["cx_list"]
            cy_list = payload["cy_list"]
            junctionflag = payload["junctionflag"]
            junctioncenter = payload["junctioncenter"]
            greendots = payload["greendots"]
            count = payload["count"]
            
            try:
                if self.streaming and self.client_socket:
                    try:
                        # Downscale the frame to target resolution before encoding
                        edited_downscaled = cv2.resize(edited, (self.target_width, self.target_height), interpolation=cv2.INTER_NEAREST)
                        
                        # Encode frame as JPEG
                        _, frame = cv2.imencode('.jpg', edited_downscaled, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
                        data = frame.tobytes()
                        size = len(data)

                        # Send header (size) then data
                        self.client_socket.sendall(struct.pack(">L", size) + data)
                    except Exception as e:
                        print(f"Error sending frame: {e}")
                        self.streaming = False
                        if not self.local_save_enabled:
                            self.local_save_enabled = True
                            self.open_files()  # fallback to local saving if streaming fails
                
                if self.local_save_enabled:
                    # Downscale all images to target resolution before saving
                    edited = cv2.resize(edited, (self.target_width, self.target_height), interpolation=cv2.INTER_NEAREST)
                    original = cv2.resize(original, (self.target_width, self.target_height), interpolation=cv2.INTER_NEAREST)
                    binary = cv2.resize(binary, (self.target_width, self.target_height), interpolation=cv2.INTER_NEAREST)
                    green = cv2.resize(green, (self.target_width, self.target_height), interpolation=cv2.INTER_NEAREST)
                    
                    if junctionflag is True:
                        self.points.write(f"{count}: {cx_list}, {cy_list}\n")
                        self.junctions_centers_file.write(f"{count}: {junctioncenter}\n")
                        self.green_dots.write(f"{count}: {greendots}\n")
                        cv2.imwrite(f"{self.path_edited_junction}/img_{count}.png", edited)
                        cv2.imwrite(f"{self.path_binary_junction}/img_{count}.png", binary)
                        cv2.imwrite(f"{self.path_original_junction}/img_{count}.png", original)
                        cv2.imwrite(f"{self.path_green_junction}/img_{count}.png", green)
                
                    elif junctionflag is False:
                        self.points.write(f"{count}: {cx_list}, {cy_list}\n")
                        self.junctions_centers_file.write(f"{count}: {junctioncenter}\n")
                        self.green_dots.write(f"{count}: {greendots}\n")
                        cv2.imwrite(f"{self.path_edited_line}/img_{count}.png", edited)
                        cv2.imwrite(f"{self.path_binary_line}/img_{count}.png", binary)
                        cv2.imwrite(f"{self.path_original_line}/img_{count}.png", original)
                        cv2.imwrite(f"{self.path_green_line}/img_{count}.png", green)
                    
            except Exception as e:
                print(f"Error saving files: {e}")
            finally:
                self.queue.task_done()

    def close_file(self) -> None:
        """Flush pending work and close sockets/files cleanly."""
        print("Waiting for image saver queue to finish...")
    
        self.queue.join()
        self.running = False
        if self.client_socket:
            self.client_socket.close()

        if self.local_save_enabled:
            try:
                self.points.close()
                self.junctions_centers_file.close()
                self.green_dots.close()
            except:
                raise Exception("Error in image saving, could not close the metadata files properly")
