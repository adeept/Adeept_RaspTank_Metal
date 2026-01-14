#!/usr/bin/env python3
# File name   : functions.py
# Description : Control Functions

import time
import threading
import os
import ultra
import Kalman_filter
import move
import RPIservo
from gpiozero import InputDevice
last_status = 0

scGear = RPIservo.ServoCtrl()
scGear.start()
TL_Speed = 40
auto_speed = 55
move.setup()

curpath = os.path.realpath(__file__)
thisPath = "/" + os.path.dirname(curpath)

def num_import_int(initial):       
	global r
	with open(thisPath+"/RPIservo.py") as f:
		for line in f.readlines():
			if(line.find(initial) == 0):
				r=line
	begin=len(list(initial))
	snum=r[begin:]
	n=int(snum)
	return n

pwm0_direction = 1
pwm0_init = num_import_int('init_pwm0 = ')
pwm0_max  = 180
pwm0_min  = 0
pwm0_pos  = pwm0_init

pwm1_direction = 1
pwm1_init = num_import_int('init_pwm1 = ')
pwm1_max  = 180
pwm1_min  = 0
pwm1_pos  = pwm1_init

pwm2_direction = 1
pwm2_init = num_import_int('init_pwm2 = ')
pwm2_max  = 180
pwm2_min  = 0
pwm2_pos  = pwm2_init

line_pin_left = 22
line_pin_middle = 27
line_pin_right = 17


class Functions(threading.Thread):
	def __init__(self, *args, **kwargs):
		self.functionMode = 'none'
		self.steadyGoal = 0

		self.scanNum = 3
		self.scanList = [0,0,0]
		self.scanPos = 1
		self.scanDir = 1
		self.rangeKeep = 30
		self.scanRange = 100
		self.scanServo = 1
		self.turnServo = 2
		self.turnWiggle = 200

		super(Functions, self).__init__(*args, **kwargs)
		self.__flag = threading.Event()
		self.__flag.clear()

	def setup(self):
		global track_line_left, track_line_middle,track_line_right
		track_line_left = InputDevice(pin=line_pin_left)
		track_line_middle = InputDevice(pin=line_pin_middle)
		track_line_right = InputDevice(pin=line_pin_right)

	def pause(self):
		self.functionMode = 'none'
		move.motorStop()
		self.__flag.clear()

	def resume(self):
		self.__flag.set()

	def automatic(self):
		self.functionMode = 'Automatic'
		self.resume()

	def trackLine(self):
		self.functionMode = 'trackLine'
		self.resume()

	def keepDistance(self):
		self.functionMode = 'keepDistance'
		self.resume()

	def trackLineProcessing(self):
		global last_status
		status_right = track_line_right.value
		status_middle = track_line_middle.value
		status_left = track_line_left.value
		current_status = (status_left << 2) | (status_middle << 1) | status_right

		if last_status == current_status:
			return

		last_status = current_status

		if status_middle == 0:
			if status_left == 0 and status_right == 1:    # 0 0 1   right
				move.trackingMove(TL_Speed,1,"right")
			elif status_left == 1 and status_right == 0:  # 1 0 0 left
				move.trackingMove(TL_Speed,1,"left")
			else:									 # 0 0 0 or 1 0 1
				move.trackingMove(TL_Speed,-1,"mid")
		else:
			if status_left == 0 and status_right == 1:	#011
				move.trackingMove(TL_Speed,1,"right")
			elif status_left == 1 and status_right == 0:	#110
				move.trackingMove(TL_Speed,1,"left")
			else:	#010 or 111
				move.trackingMove(TL_Speed,-1,"mid")
		print(status_left,status_middle,status_right)
		time.sleep(0.1)


	def distRedress(self): 
		mark = 0
		distValue = ultra.checkdist()
		while True:
			distValue = ultra.checkdist()
			if distValue > 900:
				mark +=  1
			elif mark > 5 or distValue < 900:
					break
			print(distValue)
		return round(distValue,2)

	def automaticProcessing(self):
		print('automaticProcessing')
		dist = self.distRedress()
		print(dist, "cm")
		if dist >= 40:			
			move.move(auto_speed, -1, "mid")
			time.sleep(0.2)
		elif dist > 20 and dist < 40:	
			distMid = self.distRedress()
			self.scanList[2] = distMid
			move.move(auto_speed, 1, "left")
			time.sleep(0.5)
			distLeft = self.distRedress()
			self.scanList[0] = distLeft
			if self.scanList[0] > self.scanList[2]:
				move.move(auto_speed, -1, "mid")
				time.sleep(0.2)

			move.move(auto_speed, 1, "right")
			time.sleep(1)
			distRight = self.distRedress()
			self.scanList[1] = distRight
			if self.scanList[1] > self.scanList[2]:
				move.move(auto_speed, -1, "mid")
				time.sleep(0.2)
		else:		
			move.move(auto_speed, 1, "mid")
			print("Back")
			time.sleep(0.4)

	def keepDisProcessing(self):
		distanceGet = self.distRedress()
		if distanceGet >= self.rangeKeep:
			move.move(auto_speed, -1, "mid")
		elif distanceGet < self.rangeKeep:
			move.move(auto_speed, 1, "mid")
		time.sleep(0.2)

	def functionGoing(self):
		if self.functionMode == 'none':
			self.pause()
		elif self.functionMode == 'Automatic':
			self.automaticProcessing()
		elif self.functionMode == 'trackLine':
			self.trackLineProcessing()
		elif self.functionMode == 'keepDistance':
			self.keepDisProcessing()

	def run(self):
		while 1:
			self.__flag.wait()
			self.functionGoing()
			pass


if __name__ == '__main__':
	try:
		fuc=Functions()
		fuc.setup()
		while True:
			fuc.keepDisProcessing()
	except KeyboardInterrupt:
			move.motorStop()
