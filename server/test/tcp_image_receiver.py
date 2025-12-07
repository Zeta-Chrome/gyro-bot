import socket
import struct
import threading
import time

import cv2
import numpy as np

# Configuration
DISCOVERY_PORT = 9009
DISCOVERY_MESSAGE = b"DISCOVER PC"
DISCOVERY_INTERVAL = 1.0  # seconds
FRAME_HEADER_SIZE = 10  # 4 + 4 + 2 bytes

class CameraReceiver:
    """Handles receiving frames from a single ESP32 camera"""
    def __init__(self, port, camera_id):
        self.port = port
        self.camera_id = camera_id
        self.running = False
        self.connected = False
        self.tcp_socket = None
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.receive_thread = None
        
    def receive_frames(self):
        """TCP server to receive image frames from ESP32"""
        print(f"[Camera {self.camera_id}] Starting TCP server on port {self.port}...")
        
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(('0.0.0.0', self.port))
        server_sock.listen(1)
        server_sock.settimeout(2.0)
        
        print(f"[Camera {self.camera_id}] TCP server listening on port {self.port}...")
        
        while self.running:
            try:
                print(f"[Camera {self.camera_id}] Waiting for ESP32 connection...")
                client_sock, client_addr = server_sock.accept()
                self.tcp_socket = client_sock
                self.connected = True
                
                print(f"[Camera {self.camera_id}] ESP32 connected from {client_addr}")
                client_sock.settimeout(5.0)
                
                frame_count = 0
                error_count = 0
                start_time_ms = None
                
                while self.running:
                    try:
                        # Receive frame header
                        header_data = self.recv_exact(client_sock, FRAME_HEADER_SIZE)
                        if not header_data:
                            print(f"[Camera {self.camera_id}] Connection closed by ESP32")
                            break
                        
                        # Unpack header: uint32_t frame_id, uint32_t ms, uint16_t size
                        frame_id, timestamp_ms, jpeg_size = struct.unpack('!IIH', header_data)
                        
                        if start_time_ms is None:
                            start_time_ms = timestamp_ms
                        
                        # Receive JPEG data
                        jpeg_data = self.recv_exact(client_sock, jpeg_size)
                        if not jpeg_data:
                            error_count += 1
                            print(f"[Camera {self.camera_id}] Failed to receive frame {frame_id} "
                                  f"(size: {jpeg_size} bytes) - skipping")
                            continue  # ✅ Skip frame, don't break connection 

                        # Decode JPEG
                        try:
                            frame = cv2.imdecode(np.frombuffer(jpeg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                            
                            if frame is not None:
                                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                                frame = cv2.flip(frame, 0)
                                
                                with self.frame_lock:
                                    self.latest_frame = frame
                                
                                frame_count += 1
                                if frame_count % 30 == 0:
                                    fps = frame_count / ((timestamp_ms - start_time_ms) / 1000.0) if frame_count > 0 else 0
                                    print(f"[Camera {self.camera_id}] Received {frame_count} frames | "
                                          f"Frame ID: {frame_id} | Size: {jpeg_size} bytes | FPS: {fps:.2f}")
                            else:
                                error_count += 1
                                print(f"[Camera {self.camera_id}] Failed to decode frame {frame_id}")
                                
                        except Exception as e:
                            error_count += 1
                            print(f"[Camera {self.camera_id}] Frame decode error: {e}")
                            
                    except socket.timeout:
                        continue
                    except Exception as e:
                        print(f"[Camera {self.camera_id}] Receive error: {e}")
                        break
                
                print(f"[Camera {self.camera_id}] Session ended. Received {frame_count} frames, {error_count} errors")
                client_sock.close()
                self.connected = False
                
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[Camera {self.camera_id}] TCP server error: {e}")
                self.connected = False
                time.sleep(1)
        
        server_sock.close()
        print(f"[Camera {self.camera_id}] TCP server stopped")
    
    def recv_exact(self, sock, size):
        """Receive exactly 'size' bytes from socket"""
        data = bytearray()
        while len(data) < size:
            try:
                packet = sock.recv(size - len(data))
                if not packet:
                    return None
                data.extend(packet)
            except socket.timeout:
                return None
            except Exception as e:
                return None
        return bytes(data)
    
    def get_frame(self):
        """Get the latest frame"""
        with self.frame_lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None
    
    def start(self):
        """Start receiving frames"""
        self.running = True
        self.receive_thread = threading.Thread(target=self.receive_frames, daemon=True)
        self.receive_thread.start()
    
    def stop(self):
        """Stop receiving frames"""
        self.running = False
        if self.tcp_socket:
            try:
                self.tcp_socket.close()
            except:
                pass
        if self.receive_thread:
            self.receive_thread.join(timeout=2)


class DualCameraReceiver:
    """Manages two camera receivers and displays them side by side"""
    def __init__(self, port1, port2):
        self.running = False
        self.camera1 = CameraReceiver(port1, 1)
        self.camera2 = CameraReceiver(port2, 2)
        self.discovery_thread = None
        
    def start_discovery(self):
        """Send UDP discovery broadcasts until ESP32s connect"""
        print(f"Starting UDP discovery on port {DISCOVERY_PORT}...")
        
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            while self.running:
                try:
                    udp_sock.sendto(DISCOVERY_MESSAGE, ('<broadcast>', DISCOVERY_PORT))
                    if not (self.camera1.connected and self.camera2.connected):
                        print(f"Sent discovery broadcast (Cam1: {'Connected' if self.camera1.connected else 'Waiting'}, "
                              f"Cam2: {'Connected' if self.camera2.connected else 'Waiting'})")
                    time.sleep(DISCOVERY_INTERVAL)
                except Exception as e:
                    print(f"Discovery broadcast error: {e}")
                    time.sleep(DISCOVERY_INTERVAL)
        finally:
            udp_sock.close()
            print("Discovery stopped")
    
    def create_waiting_frame(self, camera_id, connected):
        """Create a placeholder frame when no image is available"""
        waiting_img = np.zeros((480, 640, 3), dtype=np.uint8)
        status_text = f"Camera {camera_id}"
        connection_text = "Connected - Waiting for frames..." if connected else "Waiting for connection..."
        
        cv2.putText(waiting_img, status_text, (220, 200), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
        cv2.putText(waiting_img, connection_text, (120, 280), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 1)
        return waiting_img
    
    def display_streams(self):
        """Display both camera feeds side by side"""
        print("Starting dual video display...")
        cv2.namedWindow('Dual ESP32 Camera Feed', cv2.WINDOW_NORMAL)
        
        while self.running:
            # Get frames from both cameras
            frame1 = self.camera1.get_frame()
            frame2 = self.camera2.get_frame()
            
            # Create placeholder if frame is None
            if frame1 is None:
                frame1 = self.create_waiting_frame(1, self.camera1.connected)
            
            if frame2 is None:
                frame2 = self.create_waiting_frame(2, self.camera2.connected)
            
            # Ensure both frames have the same height
            h1, w1 = frame1.shape[:2]
            h2, w2 = frame2.shape[:2]
            
            if h1 != h2:
                # Resize to match heights
                target_height = max(h1, h2)
                if h1 < target_height:
                    frame1 = cv2.resize(frame1, (int(w1 * target_height / h1), target_height))
                if h2 < target_height:
                    frame2 = cv2.resize(frame2, (int(w2 * target_height / h2), target_height))
            
            # Add labels
            label_height = 30
            label1 = np.zeros((label_height, frame1.shape[1], 3), dtype=np.uint8)
            label2 = np.zeros((label_height, frame2.shape[1], 3), dtype=np.uint8)
            
            cv2.putText(label1, "Camera 1 (Port 9005)", (10, 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if self.camera1.connected else (0, 0, 255), 2)
            cv2.putText(label2, "Camera 2 (Port 9006)", (10, 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if self.camera2.connected else (0, 0, 255), 2)
            
            frame1_labeled = np.vstack([label1, frame1])
            frame2_labeled = np.vstack([label2, frame2])
            
            # Concatenate frames horizontally
            combined_frame = np.hstack([frame1_labeled, frame2_labeled])
            
            cv2.imshow('Dual ESP32 Camera Feed', combined_frame)
            
            # Check for quit key
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or key == 27:  # 'q' or ESC
                print("User requested quit")
                self.running = False
                break
        
        cv2.destroyAllWindows()
        print("Display stopped")
    
    def start(self):
        """Start all threads"""
        self.running = True
        
        # Start both camera receivers
        self.camera1.start()
        self.camera2.start()
        
        # Start discovery thread
        self.discovery_thread = threading.Thread(target=self.start_discovery, daemon=True)
        self.discovery_thread.start()
        
        # Run display in main thread
        try:
            self.display_streams()
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self.stop()
    
    def stop(self):
        """Stop all threads and clean up"""
        print("Shutting down...")
        self.running = False
        
        self.camera1.stop()
        self.camera2.stop()
        
        if self.discovery_thread:
            self.discovery_thread.join(timeout=2)
        
        print("Shutdown complete")


def main():
    print("=" * 60)
    print("Dual ESP32 Camera Receiver with UDP Discovery")
    print("=" * 60)
    print(f"Discovery Port: {DISCOVERY_PORT}")
    print("Camera 1 Port: 9005")
    print("Camera 2 Port: 9006")
    print("Press 'q' or ESC to quit")
    print("=" * 60)
    
    receiver = DualCameraReceiver(9005, 9006)
    receiver.start()


if __name__ == "__main__":
    main()
