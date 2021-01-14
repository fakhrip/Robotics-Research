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