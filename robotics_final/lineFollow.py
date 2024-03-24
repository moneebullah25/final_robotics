import socket
import time
import keyboard

TCP_IP = '192.168.43.92'
TCP_PORT = 10000
BUFFER_SIZE = 1024
connected = False

while not connected:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((TCP_IP, TCP_PORT))
        connected = True
        print("Connection successful!")
    except KeyboardInterrupt:
        print("Program terminated by user.")
    except Exception as e:
        print(f"Connection failed: {e}")
        time.sleep(1)

print(f"Connected to {TCP_IP} on PORT {TCP_PORT}")

left_motor = '100'
right_motor = '100'

values = "0,0"

junctionDetected = 0
junctionDetectedTime = time.time()

def moveForward(s: socket):
    values = left_motor + ',' + right_motor
    print("Move Forward", values)
    s.send((values + '\n').encode())
    time.sleep(0.3)

def moveLeft(s: socket):
    values = left_motor + ',' + '0'
    print("Move Left", values)
    s.send((values + '\n').encode())
    time.sleep(0.3)

def moveRight(s: socket):
    values = '0' + ',' + right_motor
    print("Move Right", values)
    s.send((values + '\n').encode())
    time.sleep(0.3)

def stop(s: socket):
    values = "0,0"
    print("Stop", values)
    s.send((values + '\n').encode())
    time.sleep(0.3)

try:
    while True:
        data = s.recv(BUFFER_SIZE).decode().strip().split("\n")
        print(data)
        for d in data[0:1]:
            try:
                sensor_values = d.split(',')
                sMostRight = abs(1 - int(sensor_values[0]))
                sMostLeft = abs(1 - int(sensor_values[1]))
                sLeft = abs(1 - int(sensor_values[2]))
                sMiddle = abs(1 - int(sensor_values[3]))
                sRight = abs(1 - int(sensor_values[4]))

                print(sMostLeft, sLeft, sMiddle, sRight, sMostRight)

                # 11111 means all sensors are on Black line
                # 00000 means all sensors are not on Black line

                if sMostLeft == 1 and sLeft == 1 and sMiddle == 1 and sRight == 1 and sMostRight == 1:
                    if time.time() > float(junctionDetectedTime + 0.5):
                        junctionDetected += 1
                        junctionDetectedTime = time.time()
                        if junctionDetected >= 3:
                            stop(s)

                if junctionDetected < 3:
                    # Follow the line
                    if sLeft == 1 and sMiddle == 1 and sRight == 1:
                        moveForward(s)
                    elif sLeft == 1 and sMiddle == 1 and sRight == 0:
                        moveLeft(s)
                    elif sLeft == 1 and sMiddle == 0 and sRight == 1:
                        stop(s) # Not Possible
                    elif sLeft == 1 and sMiddle == 0 and sRight == 0:
                        moveLeft(s)
                    elif sLeft == 0 and sMiddle == 1 and sRight == 1:
                        moveRight(s)
                    elif sLeft == 0 and sMiddle == 1 and sRight == 0:
                        moveForward(s)
                    elif sLeft == 0 and sMiddle == 0 and sRight == 1:
                        moveRight(s)
                    elif sLeft == 0 and sMiddle == 0 and sRight == 0:
                        stop(s)
                
                # TODO if 3 junctions are detected, follow the green patch
                if junctionDetected >= 3:
                    for i in range(0, 5):
                        moveForward(s)
                    stop()
                    stop()
                    stop()
                    for i in range(5):
                        moveRight(s)
                    stop()
                    stop()
                    stop()
                    for i in range(0, 5):
                        moveForward(s)

                if keyboard.is_pressed('q'):
                    break
                print("Junction Detected", junctionDetected)
                # time.sleep(0.5)
            except:
                pass
        # s.send(("0,0\n").encode()) # stop after every command 

except KeyboardInterrupt:
    print("Program terminated by user.")
finally:
    print("Disconnected")
    s.close()
