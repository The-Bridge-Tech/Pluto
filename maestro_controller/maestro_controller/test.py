import maestro
servo = maestro.Controller()
# servo.setAccel(0,4)      #set servo 0 acceleration to 4
servo.setTarget(0,7000)  #set servo to move to center position
# servo.setAccel(1,4)      #set servo 0 acceleration to 4
servo.setTarget(1,7000)
servo.setSpeed(1,10)     #set speed of servo 1
x = servo.getPosition(1) #get the current position of servo 1
servo.close()