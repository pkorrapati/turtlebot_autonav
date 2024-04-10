#run with python
#python3 webstream.py

from imutils.video import VideoStream
from flask import Response, Flask, render_template, request, jsonify
from flask_cors import CORS
import json
import threading

import imutils
import time
import cv2

outputFrame = None
lock = threading.Lock()

app = Flask(__name__)
CORS(app)

kit = ServoKit(channels=16)
i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)

pca.frequency = 50

#Constants Angles min, max and default
aMin = [40, 27]
aMax = [90 + aMin[0], 90 + aMin[1]]
adef = [45 + aMin[0], 45 + aMin[1]]

vs = VideoStream(src=0,resolution=(960, 720)).start()
time.sleep(2.0)

def limitAngle(a, min, max):
	return min if (a < min) else (max if (a > max) else a)	
    
def setServoAngle(h, v):    
	global kit
	kit.servo[0].angle = limitAngle(h, aMin[0], aMax[0])
	kit.servo[1].angle = limitAngle(v, aMin[1], aMax[1])

def detect_motion():
	global vs, outputFrame, lock
	
	while True:
		frame = vs.read()				
		with lock:
			outputFrame = frame.copy()

def generate():
	global outputFrame, lock
	while True:		
		with lock:		
			if outputFrame is None:
				continue			
			(flag, encodedImage) = cv2.imencode(".jpg", cv2.flip(outputFrame, 0))
			
			if not flag:
				continue

		yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
			bytearray(encodedImage) + b'\r\n')

@app.route("/")
def index():
	return render_template("index3.html")


@app.route("/video_feed")
def video_feed():	
	return Response(generate(),
		mimetype = "multipart/x-mixed-replace; boundary=frame")

@app.route("/getDefaults", methods=['GET'])
def getDefaults():
	return jsonify ({'aMin' : aMin, 'aMax' : aMax, 'adef' : adef })

@app.route("/setServo", methods=['POST'])
def setServo():
    h = float(request.form['h'])
    v = float(request.form['v'])
    setServoAngle(h,v)
    #setServoAngle(adef[0],v)
    
    return jsonify({'success' : True})


if __name__ == '__main__':
	setServoAngle(adef[0],adef[1])
	t = threading.Thread(target=detect_motion)
	t.daemon = True
	t.start()

	app.run(host='0.0.0.0', port=5003, debug=True,
		threaded=True, use_reloader=False, ssl_context='adhoc')

# release the video stream pointer
vs.stop()
