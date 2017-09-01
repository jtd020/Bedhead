import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

#motor 1
A1Pin = 16
A2Pin = 20
Pwm1Pin = 21

#motor 2
Bin1Pin = 6
Bin2Pin = 13
PwmPin =  19

#ADC/sound detectors
CLK  = 18
MISO = 23
MOSI = 24
CS   = 25
mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

#ultrasound
GPIO_TRIGGER = 17
GPIO_ECHO = 27

def stop():
    """Stops motor"""
    GPIO.output(A1Pin, GPIO.HIGH)
    GPIO.output(A2Pin, GPIO.HIGH)
    GPIO.output(Pwm1Pin, GPIO.HIGH)
    """Stops 2nd motor"""
    GPIO.output(Bin1Pin, GPIO.HIGH)
    GPIO.output(Bin2Pin, GPIO.HIGH)
    GPIO.output(PwmPin, GPIO.HIGH)

def forward():
    """Moves 2nd motor at full speed backward"""
    GPIO.output(Bin1Pin, GPIO.HIGH)
    GPIO.output(Bin2Pin, GPIO.LOW)
    GPIO.output(PwmPin, GPIO.HIGH)
    
    """Moves motor at full speed backward"""
    GPIO.output(A1Pin, GPIO.HIGH)
    GPIO.output(A2Pin, GPIO.LOW)
    GPIO.output(Pwm1Pin, GPIO.HIGH)
    
    
def backward():
    """Moves motor at full speed"""
    GPIO.output(A1Pin, GPIO.LOW)
    GPIO.output(A2Pin, GPIO.HIGH)
    GPIO.output(Pwm1Pin, GPIO.HIGH)

    """Moves 2nd motor at full speed """
    GPIO.output(Bin1Pin, GPIO.LOW)
    GPIO.output(Bin2Pin, GPIO.HIGH)
    GPIO.output(PwmPin, GPIO.HIGH)

    #time.sleep(.5)

def turnRight():
    """Moves motor at full speed forward"""
    GPIO.output(A1Pin, GPIO.LOW)
    GPIO.output(A2Pin, GPIO.HIGH)
    GPIO.output(Pwm1Pin, GPIO.HIGH)
    """Moves 2nd motor at full speed backward"""
    GPIO.output(Bin1Pin, GPIO.HIGH)
    GPIO.output(Bin2Pin, GPIO.LOW)
    GPIO.output(PwmPin, GPIO.HIGH)
    
def turnLeft():
    """Moves motor at full speed backward"""
    GPIO.output(A1Pin, GPIO.HIGH)
    GPIO.output(A2Pin, GPIO.LOW)
    GPIO.output(Pwm1Pin, GPIO.HIGH)
    """Moves 2nd motor at full speed """
    GPIO.output(Bin1Pin, GPIO.LOW)
    GPIO.output(Bin2Pin, GPIO.HIGH)
    GPIO.output(PwmPin, GPIO.HIGH)

def settleUltrasound():
    GPIO.output(GPIO_TRIGGER, False)
    time.sleep(2)

def distance():
    
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    valid = True
    RefTime = time.time()
    StartTime = RefTime
    # save StartTime
    while (GPIO.input(GPIO_ECHO) == 0) and (StartTime-RefTime < 0.1):
        StartTime = time.time()
    if (StartTime-RefTime >= 0.1):
        valid = False
        
    RefTime = time.time()
    StopTime = time.time()
    # save time of arrival
    while (GPIO.input(GPIO_ECHO) == 1) and (StopTime-RefTime < 0.2):
        StopTime = time.time()
    if (StopTime-RefTime >= 0.1):
        valid = False
        
    
    if (valid):
        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime

        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2
    else:
        distance = -1
        
    return distance


def setup():
    GPIO.setmode(GPIO.BCM)

    #motor
    GPIO.setup(A1Pin, GPIO.OUT)
    GPIO.setup(A2Pin, GPIO.OUT)
    GPIO.setup(Pwm1Pin, GPIO.OUT)

    #motor 2
    GPIO.setup(Bin1Pin, GPIO.OUT)
    GPIO.setup(Bin2Pin, GPIO.OUT)
    GPIO.setup(PwmPin, GPIO.OUT)

    #ultrasound
    GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
    GPIO.setup(GPIO_ECHO, GPIO.IN)

    stop()
    settleUltrasound()
    
def loop():
    nextstate = 0
    while True:
        dist = distance()
        print(dist)
        soundval = mcp.read_adc(0)
        soundval_2 = mcp.read_adc(1)
        print(mcp.read_adc(0), mcp.read_adc(1), (mcp.read_adc(0)-mcp.read_adc(1)) , (mcp.read_adc(1)-mcp.read_adc(0)) )
        state = nextstate
        
        
        # state 0: stationary
        if state == 0:
            stop()
            if soundval > 150 or soundval_2 > 150:
                nextstate = 1
                time.sleep(1)
                if (soundval-soundval_2) > 0:
##                    if:
                    turnLeft()
                    time.sleep(.5)
                    stop()
##                    elif:
##                        turnLeft()
##                        time.sleep(.2)
##                    elif:
##                        turnLeft()
##                        time.sleep(.3)
##                    elif:
##                        turnLeft()
##                        time.sleep(.4)
                elif (soundval - soundval_2) < 0:
                    turnRight()
                    time.sleep(.5)
                    stop()
            else:
                nextstate = 0
                
        # state 1: moves forward
        elif state == 1:
            forward()
            if dist < 15 and dist > .5:
                stop()
                backward()
                time.sleep(.2)
                turnRight()
                time.sleep(.2)
                stop()
                time.sleep(.2)
                forward()
                time.sleep(.5)
                stop()
                time.sleep(.2)
                turnLeft()
                time.sleep(.2)
            elif soundval > 150 or soundval_2 > 150:
                nextstate = 2
                time.sleep(1)
            else:
                nextstate = 1
                
        # state 2: moves backwards
        elif state == 2:
            backward()
            print("its going backwards")
            if soundval > 150 or soundval_2 > 150:
                nextstate = 0
                time.sleep(1)
            else:
                nextstate = 2
                
        time.sleep(.5)

def destroy():

    GPIO.output(A1Pin, GPIO.HIGH)
    GPIO.output(A2Pin, GPIO.HIGH)
    GPIO.output(Bin1Pin, GPIO.HIGH)
    GPIO.output(Bin2Pin, GPIO.HIGH)
    GPIO.cleanup()

if __name__ == '__main__':
    
    setup()
    try:
        loop()
    except KeyboardInterrupt:   # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        destroy()
