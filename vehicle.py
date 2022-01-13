from gpiozero import Robot, Motor
from math import cos, sin, pi, atan2
import pygame
import time

class car():
    # Pins where motors are connected at
    # (see Motor controller schaltplan for more)
    RightMotor = Motor(24, 4, 19)
    LeftMotor = Motor(22, 17, 18)

    # DC Motor values
    MotorMax = 1  # Full speed
    MotorMin = 0  # No speed
    MotorSpeedInitial = 0.35
    MotorSpeedHardTurn = 0.3  # Speed if we want a 90 degree turn
    FactorOffsetLeft = 1  # offset Faktor for the speed
    FactorOffsetRight = 1.16  # offset Faktor for the speed

    # Constant values:
    AngleSensorRight1 = 270
    AngleSensorRight2 = 225
    DistanceWallRight = 350
    WallLead = 1800  # parameter for sensitivity of the wall following
    DistanceWallFront = 400
    DistanceNoWallDetected = 1000  # if distance to wall right is higher than this value we go straight till we find a wall
    AngleFront1 = 160
    AngleFront2 = 200
    MaxDeviation = 0.19

    AngleBack = 330  # if an obstacle was just passed but  the right sensor sees no obstacle we turn right
    DistanceObstacleBack = 400

    running = False

    def __init__(self):
        # pygame initialization
        pygame.init()
        # pygame display mode
        screen = pygame.display.set_mode((100, 100))

    def run(self, data):

        pause = True
        brake = False
        DrivingMode = 9
        msact = 0
        msprev = 0
        ObstacleSmallOld = ObstacleSmall = 0

        self.running = True

        # used to start and stop the robot;
        # KEY p, b, ESC
        CommandValues = {'Pause': 1,
                         'Brake': 0
                         }

        print("Car ready: Press ESC to Quit / p to start or pause / b to break")

        while self.running:

            for event in pygame.event.get():
                # if key down is pressed
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_b:
                        CommandValues['Brake'] = 1

                    if event.key == pygame.K_p:
                        CommandValues['Pause'] = not CommandValues['Pause']

                    if event.key == pygame.K_ESCAPE:
                        # press esc to exit
                        print("Car: EXIT")
                        self.running = False

                if event.type == pygame.KEYUP:
                    if event.key == pygame.K_b:
                        CommandValues['Brake'] = 0

            pause = CommandValues['Pause']
            brake = CommandValues['Brake']

            if not (pause or brake):
                # normal driving mode

                # if obstacle in front car -> mode 2: Left Turn
                if self.obstacle(data, self.AngleFront1, self.AngleFront2, self.DistanceWallFront):
                    DrivingMode = 2
                    angle = 0

                    # if in a smaller range no obstacle is detected we make a short brake. otherwise the left turn is too strong
                    ObstacleSmall = self.obstacle(data, self.AngleFront1, 180, self.DistanceWallFront)
                    if not ObstacleSmall and ObstacleSmallOld:  # negative flag
                        msprev = time.time_ns() / 1000000
                    msact = time.time_ns() / 1000000
                    if msact > msprev + 100:
                        DrivingMode = 2
                    else:
                        DrivingMode = 5
                    ObstacleSmallOld = ObstacleSmall

                # if obstacle soon then do a small left turn
                elif self.obstacle(data, self.AngleFront1 +5, self.AngleFront2 -5, self.DistanceWallFront + 250):
                    DrivingMode = 6
                    angle = 0

                    # if no wall detected at the right sensor we go either straight till we find a wall or turn right for the case we just passed an obstacle
                elif not \
                (self.obstacle(data, self.AngleSensorRight1, self.AngleSensorRight1, self.DistanceNoWallDetected)):
                    if self.obstacle(data, self.AngleSensorRight1, self.AngleBack, self.DistanceObstacleBack):
                        DrivingMode = 4
                        angle = 0
                    else:
                        DrivingMode = 3
                        angle = 0

                else:
                    angle = self.getdeviation(data)
                    DrivingMode = 1

                self.drivingmode(DrivingMode, angle)

            else:
                # manual pause or brake -> do nothing
                self.setspeed(self.MotorMin, self.MotorMin)

        self.setspeed(self.MotorMin, self.MotorMin)  # stop car when not running

    def stop(self):
        self.running = False
        self.setspeed(self.MotorMin, self.MotorMin)
        pygame.quit()

        print("Car stopped")

    def setspeed(self, left, right):
        # sets speed to the value on parameter for right and left wheel

        # offset wheels
        left = left * self.FactorOffsetLeft
        right = right * self.FactorOffsetRight

        # negative value means backwards
        LeftForward = True
        RightForward = True
        if left < 0:
            LeftForward = False
            left = left * -1
        if right < 0:
            RightForward = False
            right = right * -1

        # limit the speed
        if left <= self.MotorMin:
            left = self.MotorMin

        if right <= self.MotorMin:
            right = self.MotorMin

        if left >= self.MotorMax:
            left = self.MotorMax

        if right >= self.MotorMax:
            right = self.MotorMax

        # write spped to motor
        if LeftForward:
            self.LeftMotor.forward(speed=left)
        else:
            self.LeftMotor.backward(speed=left)

        if RightForward:
            self.RightMotor.forward(speed=right)
        else:
            self.LeftMotor.backward(speed=right)

    def getdeviation(self, data):
        # get deviation angle in rad

        # prepare sensor data
        Sensor1 = Point(0, 0)
        Sensor2 = Point(0, 0)
        # Sensor 1
        angleRad = (self.AngleSensorRight1 - 180) * pi / 180.0
        Sensor1.x = data[self.AngleSensorRight1] * cos(angleRad)
        Sensor1.y = data[self.AngleSensorRight1] * sin(angleRad)
        # Sensor 2:
        angleRad = (self.AngleSensorRight1 - self.AngleSensorRight2) * pi / 180.0
        Sensor2.x = data[self.AngleSensorRight2] * cos(angleRad)
        Sensor2.y = data[self.AngleSensorRight2] * sin(angleRad)

        deviation = atan2((Sensor2.y - self.DistanceWallRight), (Sensor2.x + self.WallLead - Sensor1.y))

        # limit deviation because otherwise the speed will go too fast in critical situation
        if deviation > self.MaxDeviation:
            deviation = self.MaxDeviation

        if deviation < (-1 * self.MaxDeviation):
            deviation = (-1 * self.MaxDeviation)

        return deviation

    def drivingmode(self, mode, angle):
        # mode:
        # 1= follow wall
        # 2= turn left when wall in front
        # 3= drive forward when uncommon start position
        # 4= turn right when obstacle just passed
        # 5= pause -> wait for new data
        # 6= turn left smooth, when obstacle comes soon
        # 9= trouble situation -> stop

        if mode == 1:
            SpeedLeft = self.MotorSpeedInitial + (sin(2 * angle) * self.MotorSpeedInitial)
            SpeedRight = self.MotorSpeedInitial - (sin(2 * angle) * self.MotorSpeedInitial)

            self.setspeed(SpeedLeft, SpeedRight)

        if mode == 2:
            self.setspeed(self.MotorMin, self.MotorSpeedHardTurn)

        if mode == 3:
            self.setspeed(self.MotorSpeedInitial, self.MotorSpeedInitial)

        if mode == 4:
            self.setspeed(self.MotorSpeedHardTurn, self.MotorMin)

        if mode == 5:
            self.setspeed(self.MotorMin, self.MotorMin)

        if mode == 6:
            self.setspeed(self.MotorSpeedHardTurn / 2, self.MotorSpeedHardTurn)

        if mode == 9:
            self.setspeed(self.MotorMin, self.MotorMin)

    def obstacle(self, data, Angle1, Angle2, distance):
        # returns true when measured distance is smaller than distance
        for i in range(Angle1, Angle2 + 1):
            if data[i] > 0 and data[i] <= distance and (Angle1 != 0 or Angle2 != 0):
                # print("obstacle detected at Angle ", i, "Distance: ", data[i])
                return True
        return False

class Point():
    def __init__(self, x, y):
        self.x = x
        self.y = y

