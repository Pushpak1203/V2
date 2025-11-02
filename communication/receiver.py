import socket
from .encryption_utils import decrypt_message

def start_receiver(port=5000):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", port))
    print(f"[Receiver] Listening on port {port}")
    while True:
        data, _ = sock.recvfrom(2048)
        try:
            msg = decrypt_message(data)
            print("[Receiver] Received:", msg)
        except Exception:
            pass
