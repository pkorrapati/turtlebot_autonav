import cv2
import socket
import imagezmq

ipAddr = socket.gethostbyname(socket.gethostname())

hub = imagezmq.ImageHub(open_port='tcp://'+ ipAddr +':5555', REQ_REP=False)
# hub = imagezmq.ImageHub(open_port='tcp://localhost:5555', REQ_REP=False)
# hub = imagezmq.ImageHub()   

while True:
    (sender, frame) = hub.recv_image()
    # hub.send_reply(b'OK')

    cv2.imshow(sender, frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

cv2.destroyAllWindows()