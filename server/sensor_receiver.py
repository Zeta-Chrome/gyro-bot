"""
Enhanced sensor receiver with improved data handling
"""
import socket
import struct
import threading
import time
import numpy as np
from collections import deque
import cv2
import config


class SensorReceiver:
    def __init__(self):
        self.running = False
        
        # IMU data
        self.imu_data = deque(maxlen=config.IMU_BUFFER_SIZE)
        self.imu_lock = threading.Lock()
        
        # Ultrasound data
        self.ultrasound_data = deque(maxlen=config.ULTRASOUND_BUFFER_SIZE)
        self.ultrasound_lock = threading.Lock()
        
        # Camera data
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.camera_connected = False
        self.frame_count = 0
        
        # Mode from app
        self.current_mode = "Object detection"
        self.mode_lock = threading.Lock()
        
        # Sockets
        self.udp_socket = None
        self.tcp_socket = None
        self.mode_socket = None
        
        # Threads
        self.udp_thread = None
        self.tcp_thread = None
        self.discovery_thread = None
        self.mode_thread = None
        
        # ESP32 IP
        self.esp32_ip = None
        
    def start(self):
        """Start all receiver threads"""
        self.running = True
        
        # Start UDP receiver for IMU/Ultrasound
        self.udp_thread = threading.Thread(target=self._udp_receiver, daemon=True)
        self.udp_thread.start()
        
        # Start TCP receiver for camera
        self.tcp_thread = threading.Thread(target=self._tcp_receiver, daemon=True)
        self.tcp_thread.start()
        
        # Start mode receiver from app
        self.mode_thread = threading.Thread(target=self._mode_receiver, daemon=True)
        self.mode_thread.start()
        
        # Start discovery broadcast
        self.discovery_thread = threading.Thread(target=self._discovery_broadcast, daemon=True)
        self.discovery_thread.start()
        
        print("[RECEIVER] All threads started")
        
    def stop(self):
        """Stop all threads"""
        self.running = False
        if self.udp_socket:
            self.udp_socket.close()
        if self.tcp_socket:
            try:
                self.tcp_socket.close()
            except:
                pass
        if self.mode_socket:
            self.mode_socket.close()
                
    def _udp_receiver(self):
        """Receive IMU and ultrasound data via UDP"""
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_socket.bind(('0.0.0.0', config.UDP_IMU_PORT))
        self.udp_socket.settimeout(1.0)
        
        print(f"[UDP] Listening on port {config.UDP_IMU_PORT}")
        
        while self.running:
            try:
                data, addr = self.udp_socket.recvfrom(4096)
                
                # Auto-detect ESP32 IP
                if self.esp32_ip is None:
                    self.esp32_ip = addr[0]
                    print(f"[AUTO-DETECT] ESP32 found at {self.esp32_ip}")
                
                self._parse_sensor_data(data)
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[UDP] Error: {e}")
                    
    def _parse_sensor_data(self, data):
        """Parse batched IMU + Ultrasound data"""
        try:
            offset = 0
            
            # IMU count
            if len(data) < 1:
                return
            imu_count = data[offset]
            offset += 1
            
            # Process IMU samples
            imu_size = 4 + 6 * 4  # uint32 + 6 floats
            imu_samples = []
            
            for i in range(imu_count):
                if offset + imu_size > len(data):
                    break
                    
                timestamp_ms, ax, ay, az, gx, gy, gz = struct.unpack_from('<I6f', data, offset)
                offset += imu_size
                
                sample = {
                    'timestamp': timestamp_ms / 1000.0,
                    'ax': ax, 'ay': ay, 'az': az,
                    'gx': gx, 'gy': gy, 'gz': gz
                }
                imu_samples.append(sample)
                
                with self.imu_lock:
                    self.imu_data.append(sample)
            
            # Ultrasound count
            if offset >= len(data):
                return
            us_count = data[offset]
            offset += 1
            
            # Process ultrasound samples
            us_size = 4 + 4  # uint32 + float
            us_samples = []
            
            for i in range(us_count):
                if offset + us_size > len(data):
                    break
                    
                timestamp_ms, distance_cm = struct.unpack_from('<If', data, offset)
                offset += us_size
                
                sample = {
                    'timestamp': timestamp_ms / 1000.0,
                    'distance': distance_cm
                }
                us_samples.append(sample)
                
                with self.ultrasound_lock:
                    self.ultrasound_data.append(sample)
            
        except Exception as e:
            print(f"[PARSE] Error: {e}")
            
    def _tcp_receiver(self):
        """Receive camera frames via TCP"""
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(('0.0.0.0', config.TCP_CAMERA_PORT))
        server_sock.listen(1)
        server_sock.settimeout(2.0)
        
        print(f"[TCP] Camera server listening on 0.0.0.0:{config.TCP_CAMERA_PORT}")
        
        while self.running:
            try:
                print(f"[TCP] Waiting for ESP32 camera connection...")
                client_sock, addr = server_sock.accept()
                self.tcp_socket = client_sock
                self.camera_connected = True
                self.frame_count = 0
                print(f"[TCP] ✓ Camera connected from {addr}")
                
                client_sock.settimeout(5.0)
                
                while self.running:
                    try:
                        # Receive header (10 bytes)
                        header = self._recv_exact(client_sock, 10)
                        if not header:
                            break
                            
                        frame_id, timestamp_ms, jpeg_size = struct.unpack('!IIH', header)
                        
                        # Receive JPEG data
                        jpeg_data = self._recv_exact(client_sock, jpeg_size)
                        if not jpeg_data:
                            break
                            
                        # Decode JPEG
                        frame = cv2.imdecode(
                            np.frombuffer(jpeg_data, dtype=np.uint8),
                            cv2.IMREAD_COLOR
                        )
                        
                        if frame is not None:
                            # Flip vertically (ESP32 camera orientation)
                            frame = cv2.flip(frame, 0)
                            # Convert BGR to RGB (cv2 reads as BGR, but ESP32 sends RGB)
                            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                            
                            with self.frame_lock:
                                self.latest_frame = frame
                                self.frame_count += 1
                            
                            if self.frame_count == 1:
                                print(f"[TCP] ✓ First frame received! Shape: {frame.shape}")
                            elif self.frame_count % 50 == 0:
                                print(f"[TCP] Received {self.frame_count} frames")
                            
                    except socket.timeout:
                        continue
                    except Exception as e:
                        print(f"[TCP] Frame error: {e}")
                        break
                        
                client_sock.close()
                self.camera_connected = False
                print(f"[TCP] Camera disconnected. Total frames: {self.frame_count}")
                
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[TCP] Server error: {e}")
                    
        server_sock.close()
        
    def _mode_receiver(self):
        """Receive mode changes from mobile app"""
        self.mode_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.mode_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.mode_socket.bind(('0.0.0.0', config.UDP_MODE_PORT))
        self.mode_socket.settimeout(1.0)
        
        print(f"[MODE] Listening on port {config.UDP_MODE_PORT}")
        
        while self.running:
            try:
                data, addr = self.mode_socket.recvfrom(64)
                mode = data.decode('utf-8').strip('\x00')
                
                with self.mode_lock:
                    if mode != self.current_mode:
                        self.current_mode = mode
                        print(f"[MODE] ✓ Mode changed to: '{mode}' from {addr}")
                        
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[MODE] Error: {e}")
        
    def _recv_exact(self, sock, size):
        """Receive exactly 'size' bytes"""
        data = bytearray()
        while len(data) < size:
            try:
                packet = sock.recv(size - len(data))
                if not packet:
                    return None
                data.extend(packet)
            except:
                return None
        return bytes(data)
        
    def _discovery_broadcast(self):
        """Send UDP discovery broadcasts"""
        discovery_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        discovery_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        
        print(f"[DISCOVERY] Broadcasting on port {config.DISCOVERY_PORT}")
        
        while self.running:
            try:
                discovery_sock.sendto(b"DISCOVER_PC", ('<broadcast>', config.DISCOVERY_PORT))
                if self.esp32_ip is None:
                    print("[DISCOVERY] Sent broadcast (waiting for ESP32...)")
                time.sleep(1.0)
            except Exception as e:
                print(f"[DISCOVERY] Error: {e}")
                
        discovery_sock.close()
        
    def get_latest_imu(self, n=10):
        """Get last n IMU samples"""
        with self.imu_lock:
            return list(self.imu_data)[-n:] if len(self.imu_data) > 0 else []
            
    def get_latest_ultrasound(self, n=5):
        """Get last n ultrasound samples"""
        with self.ultrasound_lock:
            return list(self.ultrasound_data)[-n:] if len(self.ultrasound_data) > 0 else []
            
    def get_latest_frame(self):
        """Get latest camera frame"""
        with self.frame_lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None
    
    def get_current_mode(self):
        """Get current mode from app"""
        with self.mode_lock:
            return self.current_mode
    
    def is_camera_connected(self):
        """Check if camera is connected"""
        return self.camera_connected
