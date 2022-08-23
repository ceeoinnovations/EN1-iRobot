#exit()
#python3

# This would be a sample student code.
from CreateLib import Create
import time

CREATE_IP = "192.168.86.225"
CREATE_PORT = 5051

code = '''
import time

def main():
    for i in range(1000):
        print(i)
        time.sleep(1)
        
main()
'''

def main():
    MyCreate = Create('/rogers')
    MyCreate.serial_init(CREATE_IP, CREATE_PORT, 1)
    MyCreate.serial_write('2 + 2 \r\n') # direct command
    print(MyCreate.serial_read())
    MyCreate.serial_run(code)
    print(MyCreate.serial_read())
    try:
        for i in range(1000):
            time.sleep(1)
            print('iteration %d, read %s' % (i,MyCreate.serial_read()))
    except Exception as e:
        print(e)
    MyCreate.serial_abort()
    MyCreate.serial_close()
    MyCreate.close()
    print('done')
        
main()
