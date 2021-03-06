# Program on server to get video
# streams and display/process them

import cv2
import imagezmq
import time 
import numpy as np

print("waiting for connection")
imageHub = imagezmq.ImageHub()

test = cv2.imread("snapshot.png")
cv2.imshow("Camera View", test)
cv2.waitKey(1)
time.sleep(5.0)
cv2.destroyAllWindows()

while True:
    print("waiting for new frame")
    rpiName, frame = imageHub.recv_image()
    cv2.imshow("Camera View", frame)

    # image processing to find target and determine motor positions

    motor = 90.0
    actuator = 20
    solendoid = True


    # send motor positions // command to the rasp pi
    
    """
    how commands work:
        (1) float :  motor        -> location (degrees)    off  
        (2) float :  actuator     -> location (degress)    off 
        (3) int   :  solendoid    -> on off
    """

    commands = [float(motor), float(actuator), int(solendoid)]
    imageHub.send_reply(bytes(str(commands), 'UTF-8'))

    print(f'sent {motor} {actuator} {solendoid}')

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# cleanup
cv2.destroyAllWindows()
