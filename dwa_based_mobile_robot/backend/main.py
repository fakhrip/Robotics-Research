import env
import time
import robot as rbt
import simulation as smt
import b0_lib.b0RemoteApi as remote
import matplotlib.pyplot as plt


with remote.RemoteApiClient("b0RemoteApi_pythonClient", "b0RemoteApi") as client:

    ###===============================###
    #  Variable initialization          #
    ###===============================###

    var = env.Var()
    robot = rbt.Robot(var)
    sim = smt.Simulation(client, var, robot)

    doNextStep = True

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
