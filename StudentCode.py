#exit()
#python3

# This would be a sample student code.
from CreateLib import Create
import time

CREATE_IP = "192.168.86.225"
CREATE_PORT = 5051

def main():
    MyCreate = Create('/rogers')
    MyCreate.serial_init("192.168.86.225", CREATE_PORT, 1)
    try:
        for i in range(1000):
            MyCreate.serial_write('2 + 2 \r\n')
            print(i)
            #MyCreate.forward(0.5)
            #MyCreate.forward(-0.5)
            MyCreate.LED(2)
            MyCreate.turn(1.57)
            MyCreate.LED(3)
            time.sleep(1)
            print(MyCreate.serial_read())
    except Exception as e:
        print(e)
    MyCreate.serial_close()
    MyCreate.close()
        
main()
