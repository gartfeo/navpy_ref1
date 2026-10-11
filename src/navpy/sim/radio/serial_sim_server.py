import struct
import socket
import threading
import time
from multiprocessing import Process


class SerialSimServer:
    """Server that simulates the serial port."""

    def __init__(self, host='localhost', port=5000):
        self.clients = []
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((host, port))
        self.server_socket.listen()
        self.running = True
        threading.Thread(target=self.accept_clients, daemon=True).start()

    def accept_clients(self):
        while self.running:
            client_socket, _ = self.server_socket.accept()
            self.clients.append(client_socket)
            threading.Thread(target=self.handle_client, args=(client_socket,), daemon=True).start()

    def handle_client(self, client_socket):
        try:
            while self.running:
                length_prefix = client_socket.recv(4)
                if not length_prefix:
                    break
                message_length = struct.unpack('>I', length_prefix)[0]
                message = client_socket.recv(message_length)
                if not message:
                    break
                self.broadcast(message, sender_socket=client_socket)
        finally:
            client_socket.close()
            self.clients.remove(client_socket)

    def broadcast(self, message, sender_socket):
        for client in self.clients:
            if client != sender_socket:
                length_prefix = struct.pack('>I', len(message))
                client.sendall(length_prefix + message)

    def stop(self):
        self.running = False
        self.server_socket.close()

    @classmethod
    def start_server(cls):
        server = SerialSimServer()
        print("Server started.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Stopping server.")
            server.stop()


if __name__ == "__main__":
    server_process = Process(target=SerialSimServer.start_server)
    server_process.start()
    time.sleep(1)  # Give the server time to start
