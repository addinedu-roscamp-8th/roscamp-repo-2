import cv2, imutils, socket
import numpy as np
import time
import base64 #change image data to text format

BUFF_SIZE = 65536

server_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) #make udp socket

server_socket.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE) #socket option --> buffer size

host_name = socket.gethostname()

#host_ip = socket.gethostbyname(host_name) 
# ㄴ ⚠️ 127.0.1.1 local host --> cannot be used for data transformation

host_ip = '192.168.0.52' # ⭐ "internet ip" needs to be used for data transformation
print(host_ip)

port = 9999
socket_address = (host_ip, port)
server_socket.bind(socket_address)
print('Listening at:', socket_address)

vid = cv2.VideoCapture("/dev/jetcocam0")
fps, st, frames_to_count, cnt = (0,0,20,0)

while True:
    msg, client_addr = server_socket.recvfrom(BUFF_SIZE)
    print('Got Connection from', client_addr)
    # print(msg)
    WIDTH=400
    while(vid.isOpened()):
        _,frame = vid.read()
        frame = imutils.resize(frame , width=WIDTH) # change frame size by image utils
        encoded, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY,80]) # compress image
        message = base64.b64encode(buffer) # encode image to text(binary)
        server_socket.sendto(message, client_addr) # send encoded image message to client
        frame = cv2.putText(frame, 'FPS: '+str(fps), (10,40), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,0.7,(0,0,255),2)
        cv2.imshow('TRANSMITTING VIDEO', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            server_socket.close()
            break
        if cnt == frames_to_count:
            try:
                fps = round(frames_to_count/(time.time()-st))
                st = time.time()
                cnt = 0
            except:
                pass
        cnt += 1
        

