# Taken from Create examples

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
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
'''

moveOptions = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),  #  x, y, z, angle (radians)
}

speedOptions = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),        # linear and angular speeds
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
                
            elif key in speedOptions.keys():
                speed = speed * speedOptions[key][0]
                turn = turn * speedOptions[key][1]
                print('currently:\tspeed %s\tturn %s ' % (speed, turn))
                
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
