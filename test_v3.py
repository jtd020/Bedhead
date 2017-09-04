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

#ultrasound 1
GPIO_TRIGGER = 17
GPIO_ECHO = 27

#ultrasound 2
GPIO_TRIGGER_2 = 12
GPIO_ECHO_2 = 22

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
    """sets up the ultrasound sensors for initial use"""
    GPIO.output(GPIO_TRIGGER, False)
    GPIO.output(GPIO_TRIGGER_2, False)
    time.sleep(2)

def distance(GPIO_TRIGGER,GPIO_ECHO):
    """calculates the distance of objects in front of the ultrasound sensors"""
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

def avoidRight():
    """avoids obstacles to the right"""
    nextstate = 1
    timer = 0
    turnLeft()
    sensor_condition = True
    while sensor_condition:
        if distance(GPIO_TRIGGER_2, GPIO_ECHO_2) < 30:
            timer += 1
            time.sleep(.02222222)
            if distance(GPIO_TRIGGER, GPIO_ECHO) < 30:
                nextstate = 0
                sensor_condition = False
        else:
            sensor_condition = False
    forward()
    time.sleep(.1)
    turnRight()
    time.sleep(timer*.02222222)
    return nextstate

def avoidLeft():
    """avoids obstacles to the left"""
    nextstate = 1
    timer = 0
    turnRight()
    sensor_condition = True
    while sensor_condition:
        if distance(GPIO_TRIGGER, GPIO_ECHO) < 30 and distance(GPIO_TRIGGER, GPIO_ECHO) > .5:
            timer += 1
            time.sleep(.02222222)
            if distance(GPIO_TRIGGER_2, GPIO_ECHO_2) < 30 and distance(GPIO_TRIGGER_2, GPIO_ECHO_2) > .5:
                nextstate = 0
                sensor_condition = False
        else:
            sensor_condition = False
    forward()
    time.sleep(.1)
    turnLeft()
    time.sleep(timer*.02222222)
    return nextstate


def calcAngle(sound, avg, threshold):
    """calaculates (roughly) the angle the sound came from"""
    range_threshold = threshold - avg
    angle_degree = (float(range_threshold))/90.0
    range_sound = sound - avg
    return abs(90-(range_sound/angle_degree))
    
  
def calcAvg(n):
    """caclulates the average of the background noise"""
    avg = 0
    for i in range(n):
        avg += ((mcp.read_adc(1)+mcp.read_adc(0))//2)
    return avg//n

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

    #ultrasound 1
    GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
    GPIO.setup(GPIO_ECHO, GPIO.IN)

    #ultrasound 2
    GPIO.setup(GPIO_TRIGGER_2, GPIO.OUT)
    GPIO.setup(GPIO_ECHO_2, GPIO.IN)
    settleUltrasound()

def loop():
    """loops the program"""
    nextstate = 0
    avg = calcAvg(100)
    threshold = 150
    print(avg)
    while True:
        dist = distance(GPIO_TRIGGER, GPIO_ECHO)
        dist_2 = distance(GPIO_TRIGGER_2, GPIO_ECHO_2)
        print ("Measured Distance = %.1f cm" % dist)
        print("Measured Distance_2 = %.1f cm" % dist_2)        
        soundval = mcp.read_adc(1)
        soundval_2 = mcp.read_adc(0)
        print(mcp.read_adc(0), mcp.read_adc(1), (mcp.read_adc(0)-mcp.read_adc(1)) , (mcp.read_adc(1)-mcp.read_adc(0)) )
        state = nextstate
        # state 0: stationary
        if state == 0:
            stop()
            if soundval > threshold or soundval_2 > threshold:
                nextstate = 1
                time.sleep(1)
##                if (soundval-soundval_2) > 0:
##                    angle = calcAngle(soundval_2, avg, threshold)
##                    print(angle)
##                    turnRight()
##                    print(angle*.00222222)
##                    time.sleep(2*angle*.00222222)
##                    stop()    
##                   
##                elif(soundval - soundval_2) < 0:
##                    angle = calcAngle(soundval, avg, threshold)
##                    print(angle)
##                    turnLeft()
##                    print(angle*.00222222)
##                    time.sleep(2*angle*.00222222)
##                    stop()
            else:
              nextstate = 0
         
        elif state == 1:
            print("heyyyyy")
            forward()
            
            if dist < 30 and dist > 1:
                nextstate = avoidLeft()
                print("the state for avoidLeft is ", nextstate)
                
            elif dist_2 < 30 and dist_2 > 1:
                nextstate = avoidRight()
                print("the state for avoidRight is ", nextstate)

            elif soundval > threshold or soundval_2 > threshold:
                nextstate = 0
                time.sleep(1)
            else:
                nextstate = 1

        time.sleep(.5)
        
def destroy():
    """ends the program"""
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
