import math
import numpy as np
import matplotlib.pyplot as plt
import dynamic_window_approach as dwa


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

        self.var = variables

        self.__currentState = [x, y, yaw, v, omega]
        self.__targetState = np.array([0, 0, 0, 0, 0])

        self.config = dwa.Config()
        self.config.robot_type = dwa.RobotType.rectangle

        self.__obstacles = []
        self.__trajectory = []
        self.__predicted_trajectory = []

        self.isNear = False
        self.isSameYaw = False
        self.isSameSpeed = False

        self.pred_yaw = 0
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

    @property
    def targetState(self) -> list:
        return self.__targetState

    @property
    def currentState(self) -> list:
        return self.__currentState

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
        oldState = self.__currentState

        if what == "position":
            [x, y] = val
            self.__currentState = [x, y, oldState[2], oldState[3], oldState[4]]
        elif what == "rotation":
            self.__currentState = [
                oldState[0],
                oldState[1],
                val,
                oldState[3],
                oldState[4],
            ]
        elif what == "velocity":
            self.__currentState = [
                oldState[0],
                oldState[1],
                oldState[2],
                val,
                oldState[4],
            ]
        elif what == "omega":
            self.__currentState = [
                oldState[0],
                oldState[1],
                oldState[2],
                oldState[3],
                val,
            ]

    def calcPath(self, cur_state=None) -> list:
        [x, y, yaw, v, omega] = cur_state if cur_state else self.__state

        u, predicted_trajectory = dwa.dwa_control(
            np.array([x, y, yaw, v, omega]),
            self.config,
            self.__goal,
            self.__obstacles,
        )

        new_state = dwa.motion(
            np.array([x, y, yaw, v, omega]), u, self.config.dt
        )

        if cur_state == None:
            self.__targetState = new_state

        self.setPredictedTrajectory(predicted_trajectory)
        
        return new_state

    def isCoordNear(self, current, goal, thresh) -> bool:
        dist_to_goal = math.hypot(current[0] - goal[0], current[1] - goal[1])
        return dist_to_goal <= thresh

    def isFinished(self) -> bool:
        self.isCoordNear(self.__currentState, self.goal, self.config.robot_radius)

    def isStateClose(self) -> bool:
        [x, y, yaw, v, omega] = self.__currentState
        [curX, curY, curYaw, curV, curOmega] = self.__targetState

        i = self.isCoordNear([curX, curY], [x, y], 0.05)
        j = abs(curYaw - yaw) <= 0.1
        k = abs(curV - v) <= 0.1

        self.isNear = self.isNear or i
        self.isSameYaw = self.isSameYaw or j
        self.isSameSpeed = self.isSameSpeed or k

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

    def plotAllStuff(self) -> None:
        plt.cla()

        # For stopping simulation with the esc key.
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