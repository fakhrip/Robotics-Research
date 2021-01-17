import math

from geopy.distance import geodesic


class Simulation:
    def __init__(self, client, variables, robot, printLog=False):
        self.client = client
        self.var = variables
        self.robot = robot

        self.__hold = 0
        self.__triggered = 0
        self.__rotateCounter = 1
        self.__movementCounter = 1
        self.__isDanger = False
        self.__isReverse = False
        self.__isRerotating = False

        self.printLog = printLog

    def setDanger(self, danger: bool) -> None:
        self.__isDanger = danger

    def setIsRerotating(self, isRerotating: bool) -> None:
        self.__isRerotating = isRerotating

        if isRerotating:
            self.__isReverse = False
            self.__rotateCounter = 1
            self.__movementCounter = 1

    ###===============================###
    #  Helper function definition       #
    ###===============================###

    def getObject(self, name: str):
        return self.client.simxGetObjectHandle(name, self.client.simxServiceCall())

    def setAllHandles(self) -> None:
        if self.var.isHandleExist():
            return

        variables = self.var.__dict__
        for x in variables:
            [isSuccess, handle] = self.getObject(variables[x]["name"])
            if isSuccess:
                self.var.setHandle(x, handle)

        if self.printLog:
            env.printAllValues(self.var)

    def getObjectVelocity(self, handle: int, sync=False, callback=None) -> list:
        if not sync:
            [isSuccess, lv, av] = self.client.simxGetObjectVelocity(
                handle, self.client.simxServiceCall()
            )

            return [lv, av] if isSuccess else None
        else:
            self.client.simxGetObjectVelocity(
                handle, self.client.simxDefaultSubscriber(callback)
            )

        return None

    def getAbsObjectOrientation(self, handle: int, sync=False, callback=None) -> list:
        if not sync:
            [isSuccess, pos] = self.client.simxGetObjectOrientation(
                handle, -1, self.client.simxServiceCall()
            )

            return pos if isSuccess else None
        else:
            self.client.simxGetObjectOrientation(
                handle, -1, self.client.simxDefaultSubscriber(callback)
            )

        return None

    def getFloatValue(self, handle: int, param: int, sync=False, callback=None) -> list:
        if not sync:
            [isSuccess, val] = self.client.simxGetObjectFloatParameter(
                handle, param, self.client.simxServiceCall()
            )

            return val if isSuccess else None
        else:
            self.client.simxGetObjectFloatParameter(
                handle, param, self.client.simxDefaultSubscriber(callback)
            )

        return None

    def getAbsPosition(self, handle: int, sync=False, callback=None) -> list:
        if not sync:
            [isSuccess, [x, y, _]] = self.client.simxGetObjectPosition(
                handle,
                -1,
                self.client.simxServiceCall(),
            )

            return [x, y] if isSuccess else None
        else:
            self.client.simxGetObjectPosition(
                handle, -1, self.client.simxDefaultSubscriber(callback)
            )

        return None

    ###===============================###
    #  Callback function definition     #
    ###===============================###

    def rangeToTarget(self, msg):
        [isSuccess, [x, y, _]] = msg

        if not isSuccess:
            return

        targetCoord = (x, y)
        if self.printLog:
            print(f"Range to target = {geodesic(targetCoord, (0.0, 0.0)).km} m")

    def sensorStateChanged(self, msg):
        [isSuccess, state] = msg[:2]

        if not isSuccess:
            return

        isDetected = state == 1
        if isDetected:
            [*_, distance, detectedCoords] = msg

            result = self.client.simxCallScriptFunction(
                f"getObstacleCoord@{self.var.ULTRASONIC_SENSOR['name']}",
                "sim.scripttype_childscript",
                detectedCoords,
                self.client.simxServiceCall(),
            )

            isCompleted = result[0]
            if isCompleted:
                [x, y, _] = result[1]

                self.robot.updateObstacle(x, y)

                if distance < 0.30:
                    self.setDanger(True)

                if self.printLog:
                    print(f"Distance to collision = {distance}")
                    print(f"Collision Coord = {detectedCoords}")

    def robotPositionChanged(self, msg):
        [isSuccess, [x, y, _]] = msg

        if not isSuccess:
            return

        if self.printLog:
            print(f"Position state = {x},{y}")

        self.robot.updateCurrentState([x, y], "position")
        self.moveRobot()

    def robotVelocityChanged(self, msg):
        [isSuccess, val] = msg

        if not isSuccess:
            return

        if self.printLog:
            print(f"Robot linear velocity  = {val}")

        self.robot.updateCurrentState(
            (val * self.robot.config.wheel_radius), "velocity"
        )
        self.moveRobot()

    def robotRotationChanged(self, msg):
        [isSuccess, val] = msg

        if not isSuccess:
            return

        result = self.client.simxCallScriptFunction(
            f"getObjectYPR@{self.var.ROBOT['name']}",
            "sim.scripttype_childscript",
            val,
            self.client.simxServiceCall(),
        )

        isCompleted = result[0]
        if isCompleted:
            yaw = result[1][b"yaw"]

            if self.printLog:
                print(f"yaw_pitch_roll = {result[1]}")

            # The original yaw is -90deg thus we have to add it with 90deg
            # so that it match the values calculated with the algorithm
            val = yaw + math.radians(90)

            self.robot.updateCurrentState(val, "rotation")
            self.moveRobot()

    def robotRotationVelocityChanged(self, msg):
        [isSuccess, val] = msg

        if not isSuccess:
            return

        if self.printLog:
            print(f"Robot rotation velocity = {val}")

        self.robot.updateCurrentState(val, "omega")
        self.moveRobot()

    ###===============================###
    #  Simulation function definition   #
    ###===============================###

    def getRangeToTarget(self):
        self.client.simxGetObjectPosition(
            self.var.TARGET_OBJECT["handle"],
            self.var.ROBOT["handle"],
            self.client.simxDefaultSubscriber(self.rangeToTarget),
        )

    def getSensorState(self):
        self.client.simxCheckProximitySensor(
            self.var.ULTRASONIC_SENSOR["handle"],
            "sim.handle_all",
            self.client.simxDefaultSubscriber(self.sensorStateChanged),
        )

    def setMotorSpeed(self, num: int):
        self.client.simxSetJointTargetVelocity(
            self.var.BACKLEFT_MOTOR["handle"], num, self.client.simxDefaultPublisher()
        )
        self.client.simxSetJointTargetVelocity(
            self.var.BACKRIGHT_MOTOR["handle"], num, self.client.simxDefaultPublisher()
        )
        self.client.simxSetJointTargetVelocity(
            self.var.FRONTLEFT_MOTOR["handle"],
            num,
            self.client.simxDefaultPublisher(),
        )
        self.client.simxSetJointTargetVelocity(
            self.var.FRONTRIGHT_MOTOR["handle"],
            num,
            self.client.simxDefaultPublisher(),
        )

    def brake(self):
        self.setMotorSpeed(0)

    def rotateServo(self, num=0, rotation="RIGHT"):
        """
        Rotate the servo to turn right or left.

        Parameters
        ----------
        num: Int
            number of rotation in degree, max (20)
        rotation: String
            type of rotation -> "LEFT" or "RIGHT"
        """
        if rotation != "LEFT" and rotation != "RIGHT":
            return

        if num > 20:
            return

        val = math.radians(abs(num))
        val = -val if rotation == "RIGHT" else val
        self.client.simxSetJointTargetPosition(
            self.var.MOTOR_SERVO["handle"],
            val,
            self.client.simxDefaultPublisher(),
        )

    def setInitialPosition(self):
        pos = self.getAbsPosition(self.var.ROBOT["handle"])
        if pos != None and self.robot.state[0] == 0.0 and self.robot.state[1] == 0.0:
            x, y = pos[0], pos[1]
            self.robot.setState([x, y, math.radians(0), 0.0, 0.0])
            self.robot.updateCurrentState([x, y], "position")

            if self.printLog:
                print(f"robot.state = {self.robot.state}")

        pos = self.getAbsPosition(self.var.TARGET_OBJECT["handle"])
        if pos != None and self.robot.goal[0] == 0.0 and self.robot.goal[1] == 0.0:
            x, y = pos[0], pos[1]
            self.robot.setGoal(x, y)

            if self.printLog:
                print(f"robot.goal = {self.robot.goal}")

    def rotateRobot(self, yaw, reverse=False):
        # Calculate difference of yaw
        d_yaw = yaw - self.robot.currentState[2]

        # Calculate wheel rotation based on the d_yaw
        # to make the rotation degree grow exponentially
        # yeah i know its stupid
        EXP = 5
        d_yaw *= EXP

        # Make sure rotation is between 0 and -20
        rotation_deg = math.degrees(d_yaw)
        rotation_deg = max(min(rotation_deg, 0), 20)

        if reverse:
            self.rotateServo(rotation_deg, rotation=("LEFT" if d_yaw < 0 else "RIGHT"))
        else:
            self.rotateServo(rotation_deg, rotation=("RIGHT" if d_yaw < 0 else "LEFT"))

    def moveRobot(self, isForced=False):
        if isForced:
            self.robot.setState(self.robot.currentState)
            self.robot.updateTrajectory(self.robot.currentState)
            self.robot.calcPath()
        else:
            self.__triggered += 1

            # Make sure all state is updated in each tick
            if not (self.__triggered % 4 == 0):
                return

            # Check if already holding long enough
            if self.__hold > 50:
                self.__isDanger = False
                self.__hold = 0

                self.__isRerotating = False
                
                self.robot.setState(self.robot.currentState)
                self.robot.updateTrajectory(self.robot.currentState)

                self.robot.calcPath()
                return

            # Hold if danger (collision is near)
            if self.__isDanger:
                self.__hold += 1

                if self.__hold > 10:
                    self.setMotorSpeed(-2)
                else:
                    self.brake()

                self.rotateServo()
                return

            if self.__isRerotating:
                if (
                    not abs((yaw := self.robot.pred_yaw) - (cur_yaw := self.robot.currentState[2]))
                    <= 0.1
                ):
                    if self.__rotateCounter % 200 == 0:
                        self.__isReverse = True
                        self.__movementCounter = 1
                    elif self.__rotateCounter % 100 == 0:
                        self.__isReverse = False
                        self.__movementCounter = 1
                
                    if abs(math.degrees(yaw) - math.degrees(cur_yaw)) < 5:
                        self.__isReverse = False
                        self.__movementCounter = 1

                    self.rotateRobot(yaw, reverse=self.__isReverse)
                    self.__rotateCounter += 1
                    self.__movementCounter += 1

                    if self.__movementCounter > 30:
                        self.setMotorSpeed(-1 if self.__isReverse else 2)        

                else:
                    self.__isRerotating = False

                    self.robot.setState(self.robot.currentState)
                    self.robot.updateTrajectory(self.robot.currentState)

                    self.robot.calcPath()
                
                return

            cur_state = self.robot.currentState
            print(f"current = {cur_state}")

            [isClose, isAchieved] = self.robot.isStateClose()
            if isClose:
                if isAchieved:
                    self.robot.setState(self.robot.targetState)
                    self.robot.updateTrajectory(self.robot.targetState)
                else:
                    self.robot.setState(self.robot.currentState)
                    self.robot.updateTrajectory(self.robot.currentState)

                self.robot.calcPath()
            elif self.robot.missedCounter >= 10:
                self.robot.missedCounter = 0

                # Check if robot is stuck on its current state
                # usually this caused by "the impossible rotation"
                # calculated from the algorithm implementation
                if (
                    self.robot.isNear and self.robot.isSameSpeed
                ) and not self.robot.isSameYaw:
                    self.setIsRerotating(True)
                    self.robot.predictRotation()
                    return

                self.robot.setState(self.robot.currentState)
                self.robot.updateTrajectory(self.robot.currentState)

                self.robot.calcPath()

        print(f"target = {self.robot.targetState.tolist()}")

        # [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        [x, y, yaw, v, omega] = self.robot.targetState
        self.setMotorSpeed(v / self.robot.config.wheel_radius)
        self.rotateRobot(yaw)

        self.robot.plotAllStuff()