#!/usr/bin/python
#
import socketserver
import cv2
import io
import ffmpeg
import numpy as np

class VideoHandler(socketserver.BaseRequestHandler):

	def handle(self):
		self.image_shape = np.asarray((40,40,3),'uint8')
		self.image_size = np.prod(self.image_shape)

		# initialize image data
		raw_image = np.zeros(self.image_shape,'uint8')
		
		# receive and process request
		data = self.request.recv(int(self.image_size*3))
		stream = io.BytesIO(data)
		
		video = ffmpeg.input('h264.mp4')
		video

		byte = stream.read(1)
		rgb = 0
		x = 0
		y = 0
		while byte != b'':
			byte = stream.read(1)

			# convert byte string to integer
			value = int.from_bytes(byte,"big")
			if value < 10:
				continue
			raw_image[x,y,rgb] = value
			
			rgb += 1
			if rgb % 3 == 0:
				rgb = 0
				x += 1
				if x % self.image_shape[0] == 0:
					x = 0
					y += 1
	
		# display results
		cv2.imshow("video", raw_image)
		cv2.waitKey(1000)

with socketserver.TCPServer(("localhost",6002),VideoHandler,True) as server:
	server.serve_forever()