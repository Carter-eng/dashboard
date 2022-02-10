
import socket


def client_program():
    host = '192.168.1.170'
    port = 40006
    client = socket.socket()
    client.connect((host, port))
    message = str(socket.gethostname()) + "_Ground"
    client.send(message.encode())

if __name__ == '__main__':
    client_program()
