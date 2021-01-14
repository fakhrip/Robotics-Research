class Var:
    """
    List of all variable names from CoppeliaSim scene.
    """

    def __init__(self):
        self.TARGET_OBJECT = {"name": "flag", "handle": None}
        self.ROBOT = {"name": "robot_base_plane", "handle": None}
        self.MOTOR_SERVO = {"name": "servo", "handle": None}
        self.ULTRASONIC_SENSOR = {"name": "sensor", "handle": None}
        self.BACKLEFT_MOTOR = {"name": "bl_joint", "handle": None}
        self.BACKRIGHT_MOTOR = {"name": "br_joint", "handle": None}
        self.FRONTLEFT_MOTOR = {"name": "fl_joint", "handle": None}
        self.FRONTRIGHT_MOTOR = {"name": "fr_joint", "handle": None}

    def setHandle(self, name, value):
        self.__dict__[name]["handle"] = value

    def isHandleExist(self):
        variables = self.__dict__
        for x in variables:
            if variables[x]["handle"] == None:
                return False

        return True


def printAllValues(var: Var):
    variables = var.__dict__
    for x in variables:
        print(f"{x} = {variables[x]}")


def main():
    var = Var()
    printAllValues(var)


if __name__ == "__main__":
    main()