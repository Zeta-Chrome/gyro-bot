"""
Multi-port UDP/TCP server for ESP32 communication
"""
import asyncio
import io
import math
import struct
import threading

from kivy.clock import Clock
from kivy.core.image import Image as CoreImage


class MultiPortServer:
    def __init__(self, app):
        self.app = app
        self.tasks = []
        self.loop = None
        self.server_ready = False

        # UDP sockets
        self.udp_transport_control = None  # Port 9000 - send control to ESP32
        self.udp_transport_imu = None  # Port 9001 - receive IMU/ultrasound from ESP32
        self.udp_transport_cam1 = None  # Port 9004 - receive camera 1
        self.udp_transport_cam2 = None  # Port 9005 - receive camera 2
        self.udp_transport_proc = None  # Port 9006 - receive processed image

        # Latest IMU data
        self.latest_pitch = 0.0
        self.latest_distance = 0.0

        # Auto-detected ESP32 IP (from first IMU packet received)
        self.esp32_ip = None
        self.broadcast_ip = self.get_broadcast_address()

    class UDPProtocol(asyncio.DatagramProtocol):
        def __init__(self, handler):
            self.handler = handler

        def connection_made(self, transport):
            self.transport = transport

        def datagram_received(self, data, addr):
            self.handler(data, addr)

    def get_broadcast_address(self):
        """Get the broadcast address for the current network"""
        import socket

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
            return "255.255.255.255"  # Fallback to global broadcast

    def handle_imu_udp(self, data, addr):
        """Handle batched IMU + Ultrasound data from ESP32"""
        try:
            # Auto-detect ESP32 IP from first packet
            if self.esp32_ip is None:
                self.esp32_ip = addr[0]
                print(f"[AUTO-DETECT] ✓ ESP32 found at {self.esp32_ip}")

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

            print(f"[IMU] Received {len(data)} bytes, {imu_count} IMU samples")

            # Process IMU samples
            # uint32_t (4 bytes) + 6 floats (24 bytes) = 28 bytes per sample
            imu_size = 28 
            
            for i in range(imu_count):
                if offset + imu_size > len(data):
                    print(f"[IMU] Warning: Not enough data for sample {i}")
                    break

                timestamp_ms, ax, ay, az, gx, gy, gz = struct.unpack_from(
                    "<I6f", data, offset
                )
                offset += imu_size
                
                pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
                print(f"[IMU {i}] Pitch: {pitch:.2f}°")
                self.latest_pitch = pitch

            # Read ultrasound count (1 byte)
            if offset >= len(data):
                return
            us_count = data[offset]
            offset += 1

            print(f"[US] {us_count} ultrasound samples")

            # Process ultrasound samples
            # uint32_t (4 bytes) + float (4 bytes) = 8 bytes per sample
            us_size = 8  # FIX: Was incorrectly 12
            
            for i in range(us_count):
                if offset + us_size > len(data):
                    print(f"[US] Warning: Not enough data for sample {i}")
                    break

                timestamp_ms, distance_cm = struct.unpack_from("<If", data, offset)
                offset += us_size  # FIX: Now correctly adds 8 bytes
                
                print(f"[US {i}] ts={timestamp_ms} | distance: {distance_cm:.2f} cm")
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
            import traceback
            traceback.print_exc()
            print(f"[IMU] Raw data ({len(data)} bytes): {data[:60].hex()}")

    def handle_camera_udp(self, data, addr, cam_id):
        """Handle JPEG camera data via UDP"""
        try:

            def update_texture(dt):
                try:
                    buf = io.BytesIO(data)
                    texture = CoreImage(buf, ext="jpg").texture
                    if cam_id == 1:
                        self.app.cam1.texture = texture
                    else:
                        self.app.cam2.texture = texture
                except Exception as e:
                    print(f"[CAM{cam_id}] Texture error: {e}")

            Clock.schedule_once(update_texture)
        except Exception as e:
            print(f"[CAM{cam_id}] Error: {e}")

    def handle_processed_udp(self, data, addr):
        """Handle processed JPEG image via UDP"""
        try:

            def update_texture(dt):
                try:
                    buf = io.BytesIO(data)
                    texture = CoreImage(buf, ext="jpg").texture
                    self.app.proc_image.texture = texture
                except Exception as e:
                    print(f"[PROC] Texture error: {e}")

            Clock.schedule_once(update_texture)
        except Exception as e:
            print(f"[PROC] Error: {e}")

    async def send_control_udp(self, magnitude, angle, servo_angle, target_ip=None):
        """Send control data to ESP32 via UDP broadcast (port 9000)"""
        if self.udp_transport_control:
            try:
                msg = struct.pack("ffi", magnitude, angle, int(servo_angle))

                # Use detected ESP32 IP if available, otherwise broadcast
                if target_ip is None:
                    target_ip = self.esp32_ip if self.esp32_ip else self.broadcast_ip

                self.udp_transport_control.sendto(msg, (target_ip, 9000))
                print(
                    f"[CONTROL] → {target_ip}:9000 | mag={magnitude:.2f}, ang={angle:.1f}°, servo={int(servo_angle)}°"
                )
            except Exception as e:
                print(f"[CONTROL] Send error: {e}")

    async def send_settings_tcp(self, kp, ki, kd, ssid, passwd, target_ip=None):
        """Send PID + WiFi credentials via TCP to ESP32 (port 9002)"""
        try:
            # Use detected ESP32 IP if available
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

        # Control TX (any port, will send to ESP32:9000)
        self.udp_transport_control, _ = await loop.create_datagram_endpoint(
            lambda: self.UDPProtocol(lambda d, a: None),
            local_addr=("0.0.0.0", 0),
            allow_broadcast=True,
        )
        print("[UDP] ✓ Control TX ready (target ESP32:9000)")

        # IMU/Ultrasound RX (port 9001)
        self.udp_transport_imu = await self.start_udp_server(self.handle_imu_udp, 9001)

        # Camera 1 RX (port 9004)
        self.udp_transport_cam1 = await self.start_udp_server(
            lambda d, a: self.handle_camera_udp(d, a, 1), 9004
        )

        # Camera 2 RX (port 9005)
        self.udp_transport_cam2 = await self.start_udp_server(
            lambda d, a: self.handle_camera_udp(d, a, 2), 9005
        )

        # Processed Image RX (port 9006)
        self.udp_transport_proc = await self.start_udp_server(
            self.handle_processed_udp, 9006
        )

        self.server_ready = True
        print("[SERVER] ✓ All servers ready!")

        await asyncio.Event().wait()
