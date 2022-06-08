#!/usr/bin/python
#
import socketserver
import cv2
import io
import struct
from PIL import Image
import numpy as np

class VideoHandler(socketserver.BaseRequestHandler):

	def handle(self):
		self.image_shape = np.asarray((200,200,3),'uint8')
		self.image_size = np.prod(self.image_shape)

		# initialize image data
		#raw_image = np.zeros(self.image_shape,'uint8')
		
		# receive and process request
		data = self.request.recv(int(self.image_size))
		with io.BytesIO(data) as file:
			raw_image = Image.open(file)
			image = np.array(raw_image.getdata(),"uint8")
		image = np.reshape(image,self.image_shape)
		print(image)


		'''
		byte = stream.read(1)
		rgb = 0
		x = 0
		y = 0
		while byte != b'':

			# convert byte string to integer
			#value = int.from_bytes(byte,"big")
			value = struct.unpack("B",byte)[0]
			raw_image[x,y,rgb] = value
			
			rgb += 1
			if rgb % 3 == 0:
				rgb = 0
				x += 1
				if x % self.image_shape[0] == 0:
					x = 0
					y += 1

			byte = stream.read(1)
		'''

		# display results
		cv2.imshow("video", image)
		cv2.waitKey(1000)

if __name__ == "__main__":
	with socketserver.TCPServer(("localhost",6002),VideoHandler,True) as server:
		server.serve_forever()

#raspivid --nopreview -w 40 -h 40 -t 0 -fps 1 -br 75 -cd MJPEG -o tcp://127.0.0.1:6002