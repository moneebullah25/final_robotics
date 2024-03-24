import socket
import keyboard
import time
# 192.168.53.215 Fazii13
# 192.168.135.215 Begonia
# '192.168.93.215' Adeen
TCP_IP = '192.168.93.215'
TCP_PORT = 10000
connected = False

while not connected:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((TCP_IP, TCP_PORT))
        connected = True
        print("Connection successful!")
    except Exception as e:
        print(f"Connection failed: {e}")
        time.sleep(1)

print(f"Connected to {TCP_IP} on PORT {TCP_PORT}")

left_motor = '100'
right_motor = '100'

values = "0,0"

try:
    while True:
        if keyboard.is_pressed('w'):
            print("w is pressed")
            values = left_motor + ',' + right_motor
            s.send((values + '\n').encode())
            time.sleep(0.3)
        elif keyboard.is_pressed('s'):
            print("s is pressed")
            values = '0,0'  # Stop both motors for backward 
            s.send((values + '\n').encode())
            time.sleep(0.3)
        elif keyboard.is_pressed('a'):
            print("a is pressed")
            values = left_motor + ',' + '0'  # Turn left
            s.send((values + '\n').encode())
            time.sleep(0.3)
        elif keyboard.is_pressed('d'):
            print("d is pressed")
            values = '0' + ',' + right_motor  # Turn right
            s.send((values + '\n').encode())
            time.sleep(0.3)
        # s.send(("0,0\n").encode()) # stop after every command 

except KeyboardInterrupt:
    print("Program terminated by user.")
finally:
    print("Disconnected")
    s.close()
