from __future__ import print_function

import socket
import json

ip = '127.0.0.1'
port = 8008

s = socket.socket()
s.connect((ip, port))

while True:
    try:
        command = raw_input('Insert command: ')

        if command == 'takeoff':
            alt = raw_input('Inset altitude: ')
            obj = {
                'command': command,
                'alt': alt
            }
        else:
            obj = {
                'command': command
            }
        s.send(json.dumps(obj))

    except KeyboardInterrupt:
        print('Quit!')
        break

s.close()
