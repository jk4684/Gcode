import socket
# import select
#import errno
#import sys
from ui import Window


class Communication:
    HEADER_LENGTH = 10
    w = Window()
    def clientCreate(self):
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        return client_socket

    def clientConnect(self, client_socket):
        client_socket.connect((self.w.ip.text, int(self.w.port.text)))
        client_socket.setblocking(False)
        print(f'Connecting to: {self.ip.text}  on port: {self.port.text}')

    def clientSend(self, client_socket, data):
        data = data.encode('utf-8')
        data_header = f'{len(data):<{self.HEADER_LENGTH}}'.encode('utf-8')
        client_socket.send(data_header + data)

    '''
    def clientRecieve(self, client_socket, message_length):
        


    try:
        while True:
            # recieve things
            username_header = client_socket.recv(self.HEADER_LENGTH)
            if not len(username_header):
                print('connection closed by the server')
                sys.exit()
            username_length = int(username_header.decode('utf-8').strip())
            username = client_socket.recv(username_length).decode('utf-8')

            message_header = client_socket.recv(HEADER_LENGTH)
            message_length = int(message_header.decode('utf-8').strip())
            message = client_socket.recv(message_length).decode('utf-8')

            print(f'{username} > {message}')

    except IOError as e:
        if e.errno != errno.EAGAIN and e.errno != errno.EWOULDBLOCK:
            print('reading error', str(e))
            sys.exit()
        continue

        except Exception as e:
            print('General error', str(e))
            sys.exit()
            pass'''
