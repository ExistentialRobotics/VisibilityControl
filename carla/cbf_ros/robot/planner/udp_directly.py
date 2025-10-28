import socket
import time


class UDPDirectly:
    def __init__(self, local_ip: str, local_port: int):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((local_ip, local_port))

    def send(self, message, target_ip: str, target_port: int):
        return self.sock.sendto(message.encode('utf-8'), (target_ip, target_port))

    def receive(self, buffer_size: int = 1024):
        data, addr = self.sock.recvfrom(buffer_size)
        return data.decode('utf-8') if data is not None else None


if __name__ == '__main__':
    udp = UDPDirectly('127.0.0.1', 5006)
    while True:
        message = input('input:')
        udp.send(message, target_ip='127.0.0.1', target_port=5005)
