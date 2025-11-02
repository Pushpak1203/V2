from cryptography.fernet import Fernet
import os

KEY_FILE = "secret.key"

def generate_key():
    key = Fernet.generate_key()
    with open(KEY_FILE, "wb") as f:
        f.write(key)
    return key

def load_key():
    if not os.path.exists(KEY_FILE):
        return generate_key()
    with open(KEY_FILE, "rb") as f:
        return f.read()

def encrypt_message(message: str) -> bytes:
    key = load_key()
    return Fernet(key).encrypt(message.encode())

def decrypt_message(token: bytes) -> str:
    key = load_key()
    return Fernet(key).decrypt(token).decode()
