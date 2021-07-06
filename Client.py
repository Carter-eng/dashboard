
import socket


def client_program():
    # get the hostname
    host = '192.168.1.180'
    port = 6006  # initiate port no above 1024

    client = socket.socket()  # get instance
    # look closely. The bind() function takes tuple as argument
    client.connect((host, port))  # bind host address and port together

    # configure how many client the server can listen simultaneously

    message = str(socket.gethostname())
        # receive data stream. it won't accept data packet greater than 1024 bytes
    client.send(message.encode())


    client.close()  # close the connection

if __name__ == '__main__':
    client_program()
