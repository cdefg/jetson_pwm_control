import Jetson.GPIO as GPIO
import time


GPIO.setmode(GPIO.BOARD)

pwm_pin = 32
GPIO.setup(pwm_pin, GPIO.OUT, initial=GPIO.LOW)

pwm = GPIO.PWM(pwm_pin, 100)  
duty_cycle = 15.0  
pwm.start(duty_cycle)

print("[+] starting pwm wave... ")
time.sleep(3)

pwm.ChangeDutyCycle(20.0)  
pwm.ChangeFrequency(100)  

time.sleep(30)

pwm.stop()
GPIO.cleanup()

