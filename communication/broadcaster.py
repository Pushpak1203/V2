import socket, json, time
from .encryption_utils import encrypt_message

def start_broadcaster(agent_id: str, port=5000, interval=0.1):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = ("127.0.0.1", port)
    while True:
        data = {"id": agent_id, "status": "ACTIVE", "speed": 25.0}
        message = json.dumps(data)
        sock.sendto(encrypt_message(message), target)
        time.sleep(interval)
