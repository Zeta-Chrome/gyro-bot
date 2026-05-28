"""
Multi-port UDP/TCP server for ESP32 communication
"""
import asyncio
import io
import socket
import struct
import threading
import math

from kivy.clock import Clock
from kivy.core.image import Image as CoreImage
from kivy.graphics.texture import Texture


class MultiPortServer:
    def __init__(self, app):
        self.app = app
        self.tasks = []
        self.loop = None
        self.server_ready = False

        # UDP sockets
        self.udp_transport_control = None  # Port 9000 - send control to ESP32
        self.udp_transport_imu = None  # Port 9001 - receive IMU/ultrasound from ESP32
        self.udp_transport_discovery = None  # Port 9009 - send discovery broadcast
        self.udp_transport_mode = None  # Port 9010 - send mode changes to PC

        # TCP camera receiver (only proc)
        self.proc_receiver = None

        # Latest IMU data
        self.latest_pitch = 0.0
        self.latest_distance = 0.0

        # Auto-detected ESP32 IP (from first IMU packet received)
        self.esp32_ip = None
        self.broadcast_ip = None
        
        # Discovery
        self.discovery_running = False
        self.discovery_task = None
        
        # PC IP for mode updates (you can set this or auto-detect)
        self.pc_ip = None  # Will use broadcast if None

    class UDPProtocol(asyncio.DatagramProtocol):
        def __init__(self, handler):
            self.handler = handler

        def connection_made(self, transport):
            self.transport = transport

        def datagram_received(self, data, addr):
            self.handler(data, addr)

    def get_broadcast_address(self):
        """Get the broadcast address for the current network"""
        try:
            # Get local IP
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()

            # Calculate broadcast (assumes /24 network)
            parts = local_ip.split(".")
            broadcast = f"{parts[0]}.{parts[1]}.{parts[2]}.255"
            print(f"[NETWORK] Local IP: {local_ip}, Broadcast: {broadcast}")
            return broadcast
        except Exception as e:
            print(f"[NETWORK] Error getting broadcast address: {e}")
            return "255.255.255.255"

    def convert_jpeg_direct(self, jpeg_data):
        """
        Convert JPEG data to Kivy texture without any modifications.
        Direct display without flipping or color correction.
        """
        try:
            buf = io.BytesIO(jpeg_data)
            core_img = CoreImage(buf, ext="jpg")
            return core_img.texture
        except Exception as e:
            print(f"[IMAGE] Error processing: {e}")
            return None

    def handle_imu_udp(self, data, addr):
        """Handle batched IMU + Ultrasound data from ESP32"""
        try:
            # Auto-detect ESP32 IP from first packet
            if self.esp32_ip is None:
                self.esp32_ip = addr[0]
                print(f"[AUTO-DETECT] ✓ ESP32 found at {self.esp32_ip}")

                # Calculate broadcast IP for camera listening
                parts = self.esp32_ip.split(".")
                self.broadcast_ip = f"{parts[0]}.{parts[1]}.{parts[2]}.255"
                print(f"[AUTO-DETECT] ✓ Camera listen IP: {self.broadcast_ip}")

                # Start TCP camera receiver now that we know the network
                self.start_camera_receiver()

                # Update app's ESP32 IP
                def update_esp_ip(dt):
                    self.app.esp32_ip = self.esp32_ip

                Clock.schedule_once(update_esp_ip)

            offset = 0

            # Read IMU count (1 byte)
            if len(data) < 1:
                return
            imu_count = data[offset]
            offset += 1

            # Process IMU samples
            imu_size = 28  # uint32_t (4) + 6 floats (24)
            
            for i in range(imu_count):
                if offset + imu_size > len(data):
                    break

                timestamp_ms, ax, ay, az, gx, gy, gz = struct.unpack_from(
                    "<I6f", data, offset
                )
                offset += imu_size
                
                pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
                self.latest_pitch = pitch

            # Read ultrasound count (1 byte)
            if offset >= len(data):
                return
            us_count = data[offset]
            offset += 1

            # Process ultrasound samples
            us_size = 8  # uint32_t (4) + float (4)
            
            for i in range(us_count):
                if offset + us_size > len(data):
                    break

                timestamp_ms, distance_cm = struct.unpack_from("<If", data, offset)
                offset += us_size
                self.latest_distance = distance_cm

            # Update UI
            def update_ui(dt):
                if self.app.current_info == "IMU":
                    self.app.info_dropdown.header.text = (
                        f"IMU : Pitch {self.latest_pitch:.1f}°"
                    )
                elif self.app.current_info == "Distance":
                    self.app.info_dropdown.header.text = (
                        f"Distance : {self.latest_distance:.2f} cm"
                    )

            Clock.schedule_once(update_ui)

        except Exception as e:
            print(f"[IMU] Error parsing data: {e}")

    def start_camera_receiver(self):
        """Start TCP camera receiver for processed image only"""
        print("[TCP-CAM] Starting camera receiver...")
        
        # Stop discovery once ESP32 is found
        self.discovery_running = False
        
        # Processed image on port 9006
        self.proc_receiver = TCPCameraReceiver(
            9011, "proc", self.app, "0.0.0.0", self.convert_jpeg_direct
        )
        self.proc_receiver.start()

    async def send_control_udp(self, magnitude, angle, servo_angle, target_ip=None):
        """Send control data to ESP32 via UDP broadcast (port 9000)"""
        if self.udp_transport_control:
            try:
                msg = struct.pack("ffi", magnitude, angle, int(servo_angle))

                # Use detected ESP32 IP if available, otherwise broadcast
                if target_ip is None:
                    target_ip = self.esp32_ip if self.esp32_ip else self.broadcast_ip

                self.udp_transport_control.sendto(msg, (target_ip, 9000))
            except Exception as e:
                print(f"[CONTROL] Send error: {e}")

    async def send_mode_udp(self, mode):
        """Send mode change to PC via UDP (port 9010)"""
        if self.udp_transport_mode:
            try:
                # Convert mode string to bytes (null-terminated, max 32 bytes)
                mode_bytes = mode.encode('utf-8')[:31] + b'\x00'
                mode_data = mode_bytes.ljust(32, b'\x00')
                
                # Use PC IP if set, otherwise broadcast
                target_ip = self.pc_ip if self.pc_ip else self.broadcast_ip
                
                self.udp_transport_mode.sendto(mode_data, (target_ip, 9010))
                print(f"[MODE] Sent mode '{mode}' to {target_ip}:9010")
            except Exception as e:
                print(f"[MODE] Send error: {e}")

    async def send_settings_tcp(self, kp, ki, kd, ssid, passwd, target_ip=None):
        """Send PID + WiFi credentials via TCP to ESP32 (port 9002)"""
        try:
            if target_ip is None:
                target_ip = self.esp32_ip

            if target_ip is None:
                print("[TCP] ✗ ESP32 IP not detected yet. Wait for IMU data first.")
                return

            print(f"[TCP] Connecting to {target_ip}:9002...")
            reader, writer = await asyncio.wait_for(
                asyncio.open_connection(target_ip, 9002), timeout=5.0
            )

            # Pack data
            pid_data = struct.pack("fff", kp, ki, kd)
            ssid_data = ssid.encode("utf-8").ljust(32, b"\x00")[:32]
            passwd_data = passwd.encode("utf-8").ljust(64, b"\x00")[:64]

            message = pid_data + ssid_data + passwd_data

            writer.write(message)
            await writer.drain()

            print(f"[TCP] ✓ Sent settings: kp={kp}, ki={ki}, kd={kd}")
            print(f"[TCP] ✓ WiFi: SSID='{ssid}'")

            writer.close()
            await writer.wait_closed()

        except asyncio.TimeoutError:
            print(f"[TCP] ✗ Connection timeout to {target_ip}:9002")
        except Exception as e:
            print(f"[TCP] ✗ Error: {e}")

    async def start_udp_server(self, handler, port):
        """Start a UDP server on specified port"""
        loop = asyncio.get_event_loop()
        transport, protocol = await loop.create_datagram_endpoint(
            lambda: self.UDPProtocol(handler),
            local_addr=("0.0.0.0", port),
            allow_broadcast=True,
        )
        print(f"[UDP] ✓ Listening on port {port}")
        return transport

    def start_all(self):
        """Start all servers in a separate thread"""
        threading.Thread(target=self._run_all, daemon=True).start()

    def _run_all(self):
        """Run asyncio loop in thread"""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._start_all_async())

    async def _start_all_async(self):
        """Launch all async servers"""
        print("[SERVER] Starting all servers...")

        loop = asyncio.get_event_loop()

        # Get broadcast address
        self.broadcast_ip = self.get_broadcast_address()

        # Control TX (any port, will send to ESP32:9000)
        self.udp_transport_control, _ = await loop.create_datagram_endpoint(
            lambda: self.UDPProtocol(lambda d, a: None),
            local_addr=("0.0.0.0", 0),
            allow_broadcast=True,
        )
        print("[UDP] ✓ Control TX ready (target ESP32:9000)")

        # Discovery TX (send to port 9009)
        self.udp_transport_discovery, _ = await loop.create_datagram_endpoint(
            lambda: self.UDPProtocol(lambda d, a: None),
            local_addr=("0.0.0.0", 0),
            allow_broadcast=True,
        )
        print("[UDP] ✓ Discovery TX ready (target broadcast:9009)")

        # Mode TX (send to PC port 9010)
        self.udp_transport_mode, _ = await loop.create_datagram_endpoint(
            lambda: self.UDPProtocol(lambda d, a: None),
            local_addr=("0.0.0.0", 0),
            allow_broadcast=True,
        )
        print("[UDP] ✓ Mode TX ready (target PC:9010)")

        # IMU/Ultrasound RX (port 9001)
        self.udp_transport_imu = await self.start_udp_server(self.handle_imu_udp, 9001)

        self.server_ready = True
        print("[SERVER] ✓ UDP servers ready! Waiting for ESP32 to auto-detect...")
        print("[SERVER] TCP camera server will start after first IMU packet")
        
        # Start discovery broadcast
        self.discovery_running = True
        self.discovery_task = asyncio.create_task(self.send_discovery_broadcasts())

        await asyncio.Event().wait()
    
    async def send_discovery_broadcasts(self):
        """Send UDP discovery broadcasts to help ESP32s find the PC"""
        DISCOVERY_MESSAGE = b"DISCOVER_PC"
        DISCOVERY_PORT = 9009
        DISCOVERY_INTERVAL = 1.0  # seconds
        
        print(f"[DISCOVERY] Starting broadcasts to {self.broadcast_ip}:{DISCOVERY_PORT}")
        
        while self.discovery_running:
            try:
                if self.udp_transport_discovery:
                    self.udp_transport_discovery.sendto(
                        DISCOVERY_MESSAGE, 
                        (self.broadcast_ip, DISCOVERY_PORT)
                    )
                    if not self.esp32_ip:
                        print(f"[DISCOVERY] Sent broadcast (waiting for ESP32...)")
                await asyncio.sleep(DISCOVERY_INTERVAL)
            except Exception as e:
                print(f"[DISCOVERY] Error: {e}")
                await asyncio.sleep(DISCOVERY_INTERVAL)
        
        print("[DISCOVERY] Stopped (ESP32 found)")


class TCPCameraReceiver:
    """TCP receiver for camera streams"""
    FRAME_HEADER_SIZE = 10  # 4 + 4 + 2 bytes
    
    def __init__(self, port, cam_id, app, listen_ip, jpeg_converter):
        self.port = port
        self.cam_id = cam_id
        self.app = app
        self.listen_ip = listen_ip
        self.jpeg_converter = jpeg_converter
        self.running = False
        self.thread = None
        
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
            except Exception:
                return None
        return bytes(data)
    
    def receive_frames(self):
        """TCP server to receive image frames from ESP32"""
        print(f"[CAM-{self.cam_id}] Starting TCP server on {self.listen_ip}:{self.port}...")
        
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((self.listen_ip, self.port))
        server_sock.listen(1)
        server_sock.settimeout(2.0)
        
        print(f"[CAM-{self.cam_id}] TCP server listening on port {self.port}...")
        
        while self.running:
            try:
                print(f"[CAM-{self.cam_id}] Waiting for ESP32 connection...")
                client_sock, client_addr = server_sock.accept()
                
                print(f"[CAM-{self.cam_id}] ✓ ESP32 connected from {client_addr}")
                client_sock.settimeout(5.0)
                
                frame_count = 0
                
                while self.running:
                    try:
                        # Receive frame header
                        header_data = self.recv_exact(client_sock, self.FRAME_HEADER_SIZE)
                        if not header_data:
                            print(f"[CAM-{self.cam_id}] Connection closed by ESP32")
                            break
                        
                        # Unpack header: uint32_t frame_id, uint32_t ms, uint16_t size
                        frame_id, timestamp_ms, jpeg_size = struct.unpack('!IIH', header_data)
                        
                        # Receive JPEG data
                        jpeg_data = self.recv_exact(client_sock, jpeg_size)
                        if not jpeg_data:
                            print(f"[CAM-{self.cam_id}] Failed to receive frame {frame_id}")
                            continue
                        
                        # Process frame using the converter function
                        texture = self.jpeg_converter(jpeg_data)
                        
                        if texture is not None:
                            # Update UI in main thread
                            def update_texture(dt):
                                try:
                                    self.app.proc_image.texture = texture
                                except Exception as e:
                                    print(f"[CAM-{self.cam_id}] Texture assignment error: {e}")
                            
                            Clock.schedule_once(update_texture)
                            
                            frame_count += 1
                            if frame_count % 30 == 0:
                                print(f"[CAM-{self.cam_id}] Received {frame_count} frames")
                        
                    except socket.timeout:
                        continue
                    except Exception as e:
                        print(f"[CAM-{self.cam_id}] Receive error: {e}")
                        break
                
                client_sock.close()
                
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[CAM-{self.cam_id}] TCP server error: {e}")
        
        server_sock.close()
        print(f"[CAM-{self.cam_id}] TCP server stopped")
    
    def start(self):
        """Start receiving frames"""
        self.running = True
        self.thread = threading.Thread(target=self.receive_frames, daemon=True)
        self.thread.start()
    
    def stop(self):
        """Stop receiving frames"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
