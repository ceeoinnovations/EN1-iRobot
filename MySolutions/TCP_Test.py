#exit()
#python3

from TCPLib import TCPServer

CREATE_IP = "192.168.86.225"
CREATE_PORT = 5051

serial = TCPServer (CREATE_IP, CREATE_PORT, 1)
serial.read()
serial.write('2 + 2 \r\n')
reply = serial.read()
print(reply)
serial.close()
