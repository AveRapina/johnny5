import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(35, GPIO.OUT)

p=GPIO.PWM(35, 1000)
p.start(50)

raw_input('press Enter to stop')
p.stop()
GPIO.cleanup()
