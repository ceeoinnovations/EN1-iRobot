
# Stolen (and edited) from Create examples - https://github.com/iRobotEducation/create3_examples

import sys

import termios
import tty
from Subs.CreateLib import Create

msg = '''
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

anything else : stop

CTRL-C to quit
'''

moveOptions = {  #  x, y, z, angle (radians)
    'u': (1, 0, 0, 1),
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),

    'm': (-1, 0, 0, -1),  
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    settings = saveTerminalSettings()
    motion = Create('/rogers')

    speed = 0.5
    turn = 1.0
    (x, y, z, th) = (0.0,0.0,0.0,0.0)

    try:
        print(msg)
        print('currently:\tspeed %s\tturn %s ' % (speed, turn))
        
        while True:
            key = getKey(settings)
            if key in moveOptions.keys():
                x = moveOptions[key][0]
                y = moveOptions[key][1]
                z = moveOptions[key][2]
                th = moveOptions[key][3]
                
                motion.twist(x, y, z, th, speed, turn)
                
            else:
                (x, y, z, th) = (0.0,0.0,0.0,0.0)
                if (key == '\x03'):
                    break
                    
    except Exception as e:
        print(e)

    finally:
        motion.twist(0.0, 0.0, 0.0, 0.0, 0.5, 1.0)
        restoreTerminalSettings(settings)

if __name__ == '__main__':
    main()
