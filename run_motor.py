import RPi.GPIO as GPIO
import time

A1Pin = 36
A2Pin = 38
Pwm1Pin = 40

Bin1Pin = 12
Bin2Pin = 16
PwmPin =  22


def setup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(A1Pin, GPIO.OUT)
    GPIO.setup(A2Pin, GPIO.OUT)
    GPIO.setup(Pwm1Pin, GPIO.OUT)
    GPIO.setup(Bin1Pin, GPIO.OUT)
    GPIO.setup(Bin2Pin, GPIO.OUT)
    GPIO.setup(PwmPin, GPIO.OUT)

    GPIO.output(A1Pin, GPIO.HIGH)
    GPIO.output(A2Pin, GPIO.HIGH)
    GPIO.output(Pwm1Pin, GPIO.HIGH)
    GPIO.output(Bin1Pin, GPIO.HIGH)
    GPIO.output(Bin2Pin, GPIO.HIGH)
    GPIO.output(PwmPin, GPIO.HIGH)
    
def loop():
    while True:
        """Moves motor at full speed"""
        time.sleep(.5)
        GPIO.output(A1Pin, GPIO.HIGH)
        GPIO.output(A2Pin, GPIO.LOW)
        GPIO.output(Pwm1Pin, GPIO.HIGH)
        
        GPIO.output(Bin1Pin, GPIO.HIGH)
        GPIO.output(Bin2Pin, GPIO.LOW)
        GPIO.output(PwmPin, GPIO.HIGH)


def destroy():

    GPIO.output(A1Pin, GPIO.LOW)
    GPIO.output(A2Pin, GPIO.LOW)
    GPIO.output(Bin1Pin, GPIO.LOW)
    GPIO.output(Bin2Pin, GPIO.LOW)
    GPIO.cleanup()

if __name__ == '__main__':
    
    setup()
    try:
        loop()
    except KeyboardInterrupt:   # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        destroy()
