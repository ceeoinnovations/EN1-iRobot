
''' opens a single TCP port for serial communication'''
import socket, sys, time

class TCPServer():

    def __init__(self,IP,PORT, timeout = None):
        try:
            self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except socket.error:
            print('Failed to create socket')
            sys.exit()
        self.client.connect((IP,PORT))
        print('Socket Connected to ' + IP)
        self.client.settimeout(timeout)
     
    def read(self, buffer = 1024): #Read the response sent by robot upon connecting
        try:
            msg = ''
            while True:
                msg = msg + self.client.recv(buffer).decode('ascii')
                if len(msg) == buffer:
                    msg = msg + self.client.recv(buffer).decode('ascii')
                else:
                    break
        except socket.error:
            if len(msg) == 0:
                print('Failed to read data')
        return msg

    def write(self,string):
        try:
            self.client.send(bytes(string.encode()))
        except socket.error:
            print('Failed to send data')

    def write_binary(self,string):
        try:
            self.client.send(bytes(string))
        except socket.error:
            print('Failed to send data')

    def close(self):
        self.client.close()

'''
CREATE_IP = "192.168.86.225"
CREATE_PORT = 5051

serial = TCPServer (CREATE_IP, CREATE_PORT, 0)
serial.write('2 + 2 \r\n')
time.sleep(1)
reply = serial.read()
print(reply)
serial.close()
'''


