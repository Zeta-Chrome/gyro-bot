"""
Handles communication with ESP32 - receives IMU, ultrasound, and camera data
"""
import socket
import struct
import threading
import time
import numpy as np
from collections import deque
import config


class SensorReceiver:
    def __init__(self):
        self.running = False
        
        # IMU data
        self.imu_data = deque(maxlen=100)
        self.imu_lock = threading.Lock()
        
        # Ultrasound data
        self.ultrasound_data = deque(maxlen=50)
        self.ultrasound_lock = threading.Lock()
        
        # Camera data
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.camera_connected = False
        
        # Sockets
        self.udp_socket = None
        self.tcp_socket = None
        
        # Threads
        self.udp_thread = None
        self.tcp_thread = None
        self.discovery_thread = None
        
    def start(self):
        """Start all receiver threads"""
        self.running = True
        
        # Start UDP receiver for IMU/Ultrasound
        self.udp_thread = threading.Thread(target=self._udp_receiver, daemon=True)
        self.udp_thread.start()
        
        # Start TCP receiver for camera
        self.tcp_thread = threading.Thread(target=self._tcp_receiver, daemon=True)
        self.tcp_thread.start()
        
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
            for i in range(imu_count):
                if offset + imu_size > len(data):
                    break
                    
                timestamp_ms, ax, ay, az, gx, gy, gz = struct.unpack_from('I6f', data, offset)
                offset += imu_size
                
                with self.imu_lock:
                    self.imu_data.append({
                        'timestamp': timestamp_ms / 1000.0,
                        'ax': ax, 'ay': ay, 'az': az,
                        'gx': gx, 'gy': gy, 'gz': gz
                    })
            
            # Ultrasound count
            if offset >= len(data):
                return
            us_count = data[offset]
            offset += 1
            
            # Process ultrasound samples
            us_size = 4 + 4  # uint32 + float
            for i in range(us_count):
                if offset + us_size > len(data):
                    break
                    
                timestamp_ms, distance_cm = struct.unpack_from('If', data, offset)
                offset += us_size
                
                with self.ultrasound_lock:
                    self.ultrasound_data.append({
                        'timestamp': timestamp_ms / 1000.0,
                        'distance': distance_cm
                    })
                    
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
        print(f"[TCP] Make sure ESP32 is configured to connect to this port")
        
        while self.running:
            try:
                print(f"[TCP] Waiting for ESP32 camera connection on port {config.TCP_CAMERA_PORT}...")
                client_sock, addr = server_sock.accept()
                self.tcp_socket = client_sock
                self.camera_connected = True
                print(f"[TCP] ✓ Camera connected from {addr}")
                
                client_sock.settimeout(5.0)
                frame_count = 0
                
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
                            
                        # Decode JPEG data
                        import cv2
                        frame = cv2.imdecode(np.frombuffer(jpeg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                        
                        if frame is not None:
                            # cv2.imdecode returns BGR, but ESP32 sends RGB
                            # So we need to convert BGR→RGB (confusingly named COLOR_RGB2BGR does this)
                            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                            frame = cv2.flip(frame, 0)
                            
                            with self.frame_lock:
                                self.latest_frame = frame
                                
                            frame_count += 1
                            
                            if frame_count == 1:
                                print(f"[TCP] ✓ First frame decoded! Shape: {frame.shape}")
                            
                    except socket.timeout:
                        continue
                    except Exception as e:
                        print(f"[TCP] Frame error: {e}")
                        break
                        
                client_sock.close()
                self.camera_connected = False
                print(f"[TCP] Camera disconnected. Frames: {frame_count}")
                
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[TCP] Server error: {e}")
                    
        server_sock.close()
        
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
                discovery_sock.sendto(b"DISCOVER PC", ('<broadcast>', config.DISCOVERY_PORT))
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
            
    def send_control(self, magnitude, angle, servo_angle, esp32_ip='192.168.1.255'):
        """Send control commands to ESP32"""
        try:
            control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            control_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            
            msg = struct.pack('ffi', magnitude, angle, int(servo_angle))
            control_sock.sendto(msg, (esp32_ip, config.UDP_CONTROL_PORT))
            control_sock.close()
            
        except Exception as e:
            print(f"[CONTROL] Send error: {e}")
