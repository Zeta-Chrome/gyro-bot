import socket
import time

UDP_PORT = 9009      # change this if you need another port
TCP_PORT = 9006
MESSAGE = b"hello broadcast"
BROADCAST_TIME = 10   # seconds


def broadcast_udp():
    print(f"[UDP] Broadcasting for {BROADCAST_TIME} seconds on port {UDP_PORT}...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    start = time.time()
    while time.time() - start < BROADCAST_TIME:
        sock.sendto(MESSAGE, ('255.255.255.255', UDP_PORT))
        time.sleep(0.1)   # send 10 packets per second

    sock.close()
    print("[UDP] Broadcast finished.\n")


def tcp_receive():
    print(f"[TCP] Listening on port {TCP_PORT}...")

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("0.0.0.0", TCP_PORT))
    server.listen(1)

    conn, addr = server.accept()
    print(f"[TCP] Connection from {addr}")

    while True:
        data = conn.recv(4096)
        if not data:
            break
        print(f"[TCP] Received packet length: {len(data)} bytes")

    conn.close()
    server.close()
    print("[TCP] Connection closed.")


if __name__ == "__main__":
    broadcast_udp()
    tcp_receive()
