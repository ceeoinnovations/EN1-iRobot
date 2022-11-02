# cd /media/psf/Home/Documents/Work/Classes/2022\ 2Fall/EN1-iRobot/ClassNotebooks/EN1-iRobot
#exit()
#python3

# This would be a sample student code.
from CreateLib import Create
import time

def main():
    MyCreate = Create('/rogers')
    try:
        for i in range(1000):
            print(i)
            #MyCreate.forward(0.5)
            #MyCreate.forward(-0.5)
            MyCreate.LED(2)
            MyCreate.turn(1.57)
            MyCreate.LED(3)
            time.sleep(1)
    except Exception as e:
        print(e)
    MyCreate.close()
        
main()
