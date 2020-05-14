
import RPi.GPIO as GPIO
import time
import numpy as np

def check(Tr, Ec):
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(Tr, GPIO.OUT,initial=GPIO.LOW)
	GPIO.setup(Ec, GPIO.IN)
	GPIO.output(Tr, GPIO.HIGH)
	time.sleep(0.000015)
	GPIO.output(Tr, GPIO.LOW)


	while not GPIO.input(Ec):
		if KeyboardInterrupt:
			break
		pass
	t1 = time.time()

	while GPIO.input(Ec):
		if KeyboardInterrupt:
			break
		pass
	t2 = time.time()
	dist = (t2-t1)*340/2
	return dist

if __name__ == '__main__':
	# np.arange(1, 40, 1).tolist()
	for i in range(30):
		for j in  range(30):
			try: 
				dist = check(i, j)
				if dist > 0.001:
					print(i, j, dist)
			except Exception:
				pass
			

