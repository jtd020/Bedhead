import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import numpy as np

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
    #print(StartTime)
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

def avoidRight(distance_threshold, error_range):
    """avoids obstacles to the right"""
    #print("It reaches avoidRight")
    nextstate = 1
    timer = 0
    turnLeft()
    sensor_condition = True
    while sensor_condition:
        if distance(GPIO_TRIGGER, GPIO_ECHO) < distance_threshold:
            #print(distance(GPIO_TRIGGER,GPIO_ECHO))
            timer += 1
            time.sleep(.2222222)
            print("It reaches avoidRight loop")
            #print("This has been running for ", (timer)*.2222222)
            if distance(GPIO_TRIGGER_2, GPIO_ECHO_2) < distance_threshold:
                return 0
        else:
            sensor_condition = False
    stop()
    time.sleep(.5)
    forward()
    time.sleep(.75)
    stop()
    time.sleep(.5)
    turnRight()
    time.sleep((timer)*.2222222)
    stop()
    return nextstate

def avoidLeft(distance_threshold, error_range):
    """avoids obstacles to the left""" 
    #print("It reaches avoidLeft")
    nextstate = 1
    timer = 0
    turnRight()
    sensor_condition = True
    
    while sensor_condition:
        if distance(GPIO_TRIGGER_2, GPIO_ECHO_2) < distance_threshold :
            #print(distance(GPIO_TRIGGER_2, GPIO_ECHO_2))
            timer += 1
            time.sleep(.2222222)
            print("It reaches avoidLeft loop")
            #print("This has been running for ", (timer)*.2222222)
            if distance(GPIO_TRIGGER, GPIO_ECHO) < distance_threshold :
                return 0
        else:
            sensor_condition = False

    stop()
    time.sleep(.5)
    forward()
    time.sleep(.75)
    stop()
    time.sleep(.5)
    turnLeft()
    time.sleep((timer)*.2222222)
    stop()
    return nextstate

def time_difference(soundval, soundval_2, threshold, max_time):
    detected_1st =  False
    detected_2nd = False
    time_sensor1 = 0
    time_sensor2 = 0
    start_time = time.time()
    current_time = time.time()
    print("IIII")
    while (not (detected_1st and detected_2nd)) and (current_time - start_time < max_time):
        sound1_condition = (soundval > threshold)
        sound2_condition = (soundval_2 > threshold)
        if (sound1_condition):
            detected_1st = True
            time_sensor1 = time.time()
            print("time_sensor1 = ",time_sensor1)
        if (sound2_condition):
            detected_2nd = True
            time_sensor2 = time.time()
            print("time_sensor2 = ",time_sensor2)
        current_time = time.time()
        #print("current time is ", current_time)
        #print(current_time-time_sensor1)
        #print(current_time-time_sensor2)

    if (detected_1st and detected_2nd):
        return (current_time-time_sensor1) - (current_time-time_sensor2)
    else:
        return 0

def calcAngle(sensor_distance, time_diff):
    """calculates (roughly) the angle the sound came from. Distances are measured in CENTIMETERS"""
    spd_o_sound = 34300
    sin_angle = (time_diff*spd_o_sound)/sensor_distance
    angle = np.arcsin(sin_angle)
    angle = angle*(180/3.14)
    return angle

def calcAvg(n):
    """caclulates the average of the background noise"""
    avg = 0
    for i in range(n):
        avg += ((mcp.read_adc(1)+mcp.read_adc(0))/2)
    return avg/n

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
    #state of loop
    nextstate = 0
    #distance between the two sensors
    sensor_distance = 20 
    #maximum time for sound detectors to pick up sound
    max_time =  5
    #thresholf of ultrasound detectors
    distance_threshold = 20
    #error range of ultrasound detectors
    error_range = .5
    #time.sleep wait time for angle
    wait_time_angle = .01
    avg = calcAvg(5)
    overall_avg = avg
    print(overall_avg)
    counter = 1
    while True:
        dist = distance(GPIO_TRIGGER, GPIO_ECHO)
        dist_2 = distance(GPIO_TRIGGER_2, GPIO_ECHO_2)
        threshold = overall_avg + 10
        print ("Measured Distance = %.1f cm" % dist)
        print("Measured Distance_2 = %.1f cm" % dist_2)        
        soundval = mcp.read_adc(1)
        soundval_2 = mcp.read_adc(0)
        print(mcp.read_adc(0), mcp.read_adc(1))
        state = nextstate
        avg = calcAvg(5)
        time_sensor1 = 0
        time_sensor2 = 0
        detected_1st =  False
        detected_2nd = False
        # state 0: stationary
        if state == 0:
            start_time = time.time()
            current_time = time.time()
            while (not (detected_1st and detected_2nd)) and (current_time - start_time < max_time):
                sound1_condition = (soundval > threshold)
                sound2_condition = (soundval_2 > threshold)
                if (sound2_condition and not detected_2nd):
                    detected_2nd = True
                    time_sensor2 = time.time()
                    print("time_sensor2 = ",time_sensor2)
                    
                    if (sound1_condition and not detected_1st):
                        detected_1st = True
                        time_sensor1 = time.time()
                        print("time_sensor1 = ",time_sensor1)
                if (sound1_condition and not detected_1st):
                    detected_1st = True
                    time_sensor1 = time.time()
                    print("time_sensor1 = ",time_sensor1)
                        
                    if (sound2_condition and not detected_2nd):
                        detected_2nd = True
                        time_sensor2 = time.time()
                        print("time_sensor2 = ",time_sensor2)
                    
                current_time = time.time()
                #print("current time is ", current_time)
                #print(current_time-time_sensor1)
                #print(current_time-time_sensor2)

            if (detected_1st and detected_2nd):
                time_diff = abs((current_time-time_sensor1) - (current_time-time_sensor2))
                angle = calcAngle(sensor_distance, time_diff)
                print("the angle is ",angle)
                if time_sensor1 < time_sensor2:
                    turnRight()
                    time.sleep(angle * wait_time_angle) 
                    stop()
                if time_sensor1 > time_sensor2:
                    print("it's turning left")
                    turnLeft()
                    time.sleep(angle * wait_time_angle) 
                    stop()
            else:
                nextstate = 0

        elif state == 1:
            #print("heyyyyy")
            forward()
            
            if dist < distance_threshold and dist > 1:
                nextstate = avoidRight(distance_threshold,error_range)
                
            elif dist_2 < distance_threshold and dist_2 > 1:
                nextstate = avoidLeft(distance_threshold,error_range)

            else:
                nextstate = 1
                
        overall_avg = ((overall_avg * counter) + avg)/(counter + 1)
        if counter < 20:
            counter += 1
            
        
def destroy():
    """ends the program"""
    GPIO.output(A1Pin, GPIO.HIGH)
    GPIO.output(A2Pin, GPIO.HIGH)
    GPIO.output(Bin1Pin, GPIO.HIGH)
    GPIO.output(Bin2Pin, GPIO.HIGH)

if __name__ == '__main__':
    
    setup()
    try:
        loop()
    except KeyboardInterrupt:   # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        destroy()
