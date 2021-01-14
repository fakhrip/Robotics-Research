import time
import math
import env
import b0RemoteApi

import numpy as np
import matplotlib.pyplot as plt
import dynamic_window_approach as dwa

from geopy.distance import geodesic


class Robot:
    def __init__(
        self,
        variables,
        x=0.0,
        y=0.0,
        yaw=math.radians(0),
        v=0.0,
        omega=0.0,
    ):
        self.setState([x, y, yaw, v, omega])
        self.setGoal(0, 0)

        self.currentState = [x, y, yaw, v, omega]
        self.targetState = np.array([0, 0, 0, 0, 0])
        self.var = variables

        self.config = dwa.Config()
        self.config.robot_type = dwa.RobotType.rectangle
        self.__predicted_trajectory = []
        self.__trajectory = []
        self.__obstacles = []

        self.isNear = False
        self.isSameSpeed = False
        self.isSameYaw = False

        self.missedCounter = 0

    @property
    def state(self) -> list:
        return self.__state

    @property
    def goal(self) -> list:
        return self.__goal

    @property
    def trajectory(self) -> list:
        return self.__trajectory

    @property
    def obstacles(self) -> list:
        return self.__obstacles

    def setState(self, arr: list) -> None:
        self.__state = np.array(arr)

    def setPredictedTrajectory(self, arr: list) -> None:
        self.__predicted_trajectory = arr

    def setGoal(self, x: float, y: float) -> None:
        self.__goal = np.array([x, y])

    def updateTrajectory(self, state: list) -> None:
        if len(self.__trajectory) > 0:
            self.__trajectory = np.vstack((self.__trajectory, state))
        else:
            self.__trajectory = np.array(self.__state)

    def updateObstacle(self, x: float, y: float) -> None:
        if len(self.__obstacles) > 0:
            self.__obstacles = np.concatenate((self.__obstacles, np.array([[x, y]])))
        else:
            self.__obstacles = np.array([[x, y]])

    def updateCurrentState(self, val, what: int) -> None:

        oldState = self.currentState

        if what == "position":
            [x, y] = val
            self.currentState = [x, y, oldState[2], oldState[3], oldState[4]]
        elif what == "rotation":
            self.currentState = [
                oldState[0],
                oldState[1],
                val,
                oldState[3],
                oldState[4],
            ]
        elif what == "velocity":
            self.currentState = [
                oldState[0],
                oldState[1],
                oldState[2],
                val,
                oldState[4],
            ]
        elif what == "omega":
            self.currentState = [
                oldState[0],
                oldState[1],
                oldState[2],
                oldState[3],
                val,
            ]

    def calcPath(self) -> list:
        u, predicted_trajectory = dwa.dwa_control(
            self.__state, self.config, self.__goal, self.__obstacles
        )

        new_state = dwa.motion(self.__state, u, self.config.dt)

        self.targetState = new_state
        self.setPredictedTrajectory(predicted_trajectory)

        return new_state

    def isCoordNear(self, current, goal, thresh) -> bool:
        dist_to_goal = math.hypot(current[0] - goal[0], current[1] - goal[1])
        return dist_to_goal <= thresh

    def isFinished(self) -> bool:
        self.isCoordNear(self.currentState, self.goal, self.config.robot_radius)

    def isStateClose(self) -> bool:
        [x, y, yaw, v, omega] = self.currentState
        [curX, curY, curYaw, curV, curOmega] = self.targetState

        i = self.isCoordNear([curX, curY], [x, y], 0.05)
        j = abs(curV - v) <= 0.1
        k = abs(curYaw - yaw) <= 0.1

        self.isNear = True if self.isNear else i
        self.isSameYaw = True if self.isSameYaw else j
        self.isSameSpeed = True if self.isSameSpeed else k

        if self.isNear and self.isSameYaw and self.isSameSpeed:
            self.isNear = False
            self.isSameYaw = False
            self.isSameSpeed = False
            self.missedCounter = 0

            if i and j and k:
                return [True, True]

            return [True, False]
        else:
            self.missedCounter += 1
            return [False, False]

    def plotAllStuff(self):
        plt.cla()

        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            "key_release_event",
            lambda event: [exit(0) if event.key == "escape" else None],
        )

        if len(self.__predicted_trajectory) > 0:
            plt.plot(
                self.__predicted_trajectory[:, 0],
                self.__predicted_trajectory[:, 1],
                "-g",
            )

        if len(self.__obstacles) > 0:
            plt.plot(self.__obstacles[:, 0], self.__obstacles[:, 1], "ok")

        plt.plot(self.__state[0], self.__state[1], "xr")
        plt.plot(self.__goal[0], self.__goal[1], "xb")

        dwa.plot_robot(self.__state[0], self.__state[1], self.__state[2], self.config)
        dwa.plot_arrow(self.__state[0], self.__state[1], self.__state[2], length=0.3)

        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.1)


class Simulation:
    def __init__(self, client, variables, robot):
        self.client = client
        self.var = variables
        self.robot = robot

        self.__triggered = 0
        self.__hold = 0
        self.__isDanger = False

    def setDanger(self, danger: bool) -> None:
        self.__isDanger = danger

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

        if printLog:
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
        if printLog:
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

                if printLog:
                    print(f"Distance to collision = {distance}")
                    print(f"Collision Coord = {detectedCoords}")

    def robotPositionChanged(self, msg):
        [isSuccess, [x, y, _]] = msg

        if not isSuccess:
            return

        if printLog:
            print(f"Position state = {x},{y}")

        self.robot.updateCurrentState([x, y], "position")
        self.moveRobot()

    def robotVelocityChanged(self, msg):
        [isSuccess, val] = msg

        if not isSuccess:
            return

        if printLog:
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

            if printLog:
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

        if printLog:
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
            self.robot.currentState = [x, y, math.radians(0), 0.0, 0.0]

            if printLog:
                print(f"robot.state = {self.robot.state}")

        pos = self.getAbsPosition(self.var.TARGET_OBJECT["handle"])
        if pos != None and self.robot.goal[0] == 0.0 and self.robot.goal[1] == 0.0:
            x, y = pos[0], pos[1]
            self.robot.setGoal(x, y)

            if printLog:
                print(f"robot.goal = {self.robot.goal}")

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

                self.robot.setState(self.robot.currentState)
                self.robot.updateTrajectory(self.robot.currentState)

                self.robot.calcPath()
                return

            # Hold if danger (collision is near)
            if self.__isDanger:
                self.__hold += 1

                if self.__hold > 10:
                    self.setMotorSpeed(-3)
                else:
                    self.brake()

                self.rotateServo()
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
                self.robot.setState(self.robot.currentState)
                self.robot.updateTrajectory(self.robot.currentState)

                self.robot.calcPath()

        print(f"target = {self.robot.targetState.tolist()}")

        # [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        [x, y, yaw, v, omega] = self.robot.targetState
        self.setMotorSpeed(v / self.robot.config.wheel_radius)

        # Calculate difference of yaw
        d_yaw = yaw - self.robot.currentState[2]

        # Calculate wheel rotation based on the d_yaw
        # to make the rotation degree grow exponentially 
        # yeah i know its stupid
        EXP = 10
        rotation_deg = d_yaw * EXP

        # Make sure rotation is between 0 and -20
        rotation_deg = max(min(rotation_deg, 0), 20)

        self.rotateServo(rotation_deg, rotation=("RIGHT" if d_yaw < 0 else "LEFT"))

        self.robot.plotAllStuff()


with b0RemoteApi.RemoteApiClient("b0RemoteApi_pythonClient", "b0RemoteApi") as client:

    ###===============================###
    #  Variable initialization          #
    ###===============================###

    var = env.Var()
    robot = Robot(var)
    sim = Simulation(client, var, robot)

    doNextStep = True
    printLog = False

    ###===============================###
    #  Callback function definition     #
    ###===============================###

    def simulationStepStarted(msg):
        simTime = msg[1][b"simulationTime"]
        print(f"\nSimulation step started. Simulation time: {simTime}")

        # if robot.isFinished():
        #     print("Done")
        #     client.simxStopSimulation(client.simxDefaultPublisher())

    def simulationStepDone(msg):
        simTime = msg[1][b"simulationTime"]
        print(f"Simulation step done. Simulation time: {simTime}\n")

        global doNextStep
        doNextStep = True

    ###===============================###
    #  System initialization            #
    ###===============================###

    sim.setAllHandles()
    sim.brake()
    sim.rotateServo()
    sim.setInitialPosition()

    ###===============================###
    #  Main simulation code             #
    ###===============================###

    client.simxSynchronous(True)

    sim.getRangeToTarget()
    sim.getSensorState()

    sim.getAbsPosition(
        var.ROBOT["handle"], sync=True, callback=sim.robotPositionChanged
    )
    sim.getAbsObjectOrientation(
        var.ROBOT["handle"], sync=True, callback=sim.robotRotationChanged
    )

    # check https://www.coppeliarobotics.com/helpFiles/en/objectParameterIDs.htm
    # for all available parameters
    sim.getFloatValue(
        var.BACKLEFT_MOTOR["handle"],
        2012,
        sync=True,
        callback=sim.robotVelocityChanged,
    )
    sim.getFloatValue(
        var.MOTOR_SERVO["handle"],
        2012,
        sync=True,
        callback=sim.robotRotationVelocityChanged,
    )

    sim.moveRobot(isForced=True)

    ###===============================###
    #  Synchronous simulation process   #
    ###===============================###

    client.simxGetSimulationStepStarted(
        client.simxDefaultSubscriber(simulationStepStarted)
    )
    client.simxGetSimulationStepDone(client.simxDefaultSubscriber(simulationStepDone))
    client.simxStartSimulation(client.simxDefaultPublisher())

    startTime = time.time()
    while time.time() < startTime + 100:
        if doNextStep:
            doNextStep = False
            client.simxSynchronousTrigger()
        client.simxSpinOnce()

    client.simxStopSimulation(client.simxDefaultPublisher())


# Finishing plot (show all trajectory history)
plt.plot(robot.trajectory[:, 0], robot.trajectory[:, 1], "-r")
plt.pause(0.1)
plt.show()
