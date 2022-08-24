# This would be a sample student code.
from CreateLib import Create
import time

def main():
    MyCreate = Create('/JonSnow')
    try:
        for i in range(1000):
            print(i)
            #MyCreate.forward(0.5)
            #MyCreate.forward(-0.5)
            MyCreate.LED(2)
            MyCreate.turn(1.57)
            time.sleep(5)
            MyCreate.LED(3)
            time.sleep(1)
    finally:    
        MyCreate.close()
        
main()