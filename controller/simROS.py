#!/usr/bin/env python
import rospy
import time
import sys
import signal
from std_msgs.msg import Bool, Int32, Float32, Float32MultiArray


class SIMROS(object):
    def __init__(self, argv):
        # Setup variables
        self._counter            = 0
        self.simulationTime      = 0.0
        self.simState            = 0
        self.terminateSimulation = False
        self.simStepDone         = False

        self.testParameters      = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.headingDirection    = 0
        self.stability           = 0
        self.distance            = 0
        self.meanPower           = 0
        self.meanVel             = 0
        self.jointPositions      = [0, 0, 0, 0, 0, 0, 0, 0]
        self.jointTorques        = [0, 0, 0, 0, 0, 0, 0, 0]
        self.jointVelocities     = [0, 0, 0, 0, 0, 0, 0, 0]

        # Get topic names from simulation
        MotorTopic              = "/"+argv[0]
        simulationTimeTopic     = "/"+argv[1]
        terminateNodeTopic      = "/"+argv[2]
        startSimTopic           = "/"+argv[3]
        pauseSimTopic           = "/"+argv[4]
        stopSimTopic            = "/"+argv[5]
        enableSyncModeTopic     = "/"+argv[6]
        triggerNextStepTopic    = "/"+argv[7]
        simulationStepDoneTopic = "/"+argv[8]
        simulationStateTopic    = "/"+argv[9]
        plotterTopic            = "/"+argv[10]
        jointPositionTopic      = "/"+argv[11]
        jointTorqueTopic        = "/"+argv[12]
        jointVelocityTopic      = "/"+argv[13]
        testParametersTopic     = "/"+argv[14]

        # Create a ROS node. The name has a random component:
        rospy.init_node('matcrawler')

        # Subscribe to topics and specify callback functions
        self.subSimulationTimeSub    = rospy.Subscriber(simulationTimeTopic, Float32, self.simulationTimeCallback)
        self.subTerminateNodeSub     = rospy.Subscriber(terminateNodeTopic, Bool, self.terminateNodeCallback)
        self.simulationStepDoneSub   = rospy.Subscriber(simulationStepDoneTopic, Bool, self.simulationStepDoneCallback)
        self.simulationStateSub      = rospy.Subscriber(simulationStateTopic, Int32, self.simulationStateCallback)
        self.jointPositionSub        = rospy.Subscriber(jointPositionTopic, Float32MultiArray, self.jointPositionCallback)
        self.jointTorqueSub          = rospy.Subscriber(jointTorqueTopic, Float32MultiArray, self.jointTorqueCallback)
        self.jointVelocitySub        = rospy.Subscriber(jointVelocityTopic, Float32MultiArray, self.jointVelocityCallback)
        self.testParametersSub       = rospy.Subscriber(testParametersTopic, Float32MultiArray, self.testParametersCallback)

        # Initialize publishers
        self.MotorPositionPub        = rospy.Publisher(MotorTopic, Float32MultiArray, queue_size=1)
        self.plotterPub              = rospy.Publisher(plotterTopic, Float32MultiArray, queue_size=1)
        self.startSimPub             = rospy.Publisher(startSimTopic, Int32, queue_size=1)
        self.pauseSimPub             = rospy.Publisher(pauseSimTopic, Int32, queue_size=1)
        self.stopSimPub              = rospy.Publisher(stopSimTopic, Bool, queue_size=1)
        self.enableSyncModePub       = rospy.Publisher(enableSyncModeTopic, Bool, queue_size=1)
        self.triggerNextStepPub      = rospy.Publisher(triggerNextStepTopic, Bool, queue_size=1)

        time.sleep(0.1)

    def simulationTimeCallback(self, data):
        self.simulationTime = data.data

    def terminateNodeCallback(self, data):
        self.terminateSimulation = data.data

    def simulationStepDoneCallback(self, data):
        self.simStepDone = data.data

    def simulationStateCallback(self, data):
        self.simState = data.data

    def simulationState(self):
        return self.simState

    def jointPositionCallback(self, data):
        self.jointPositions = data.data

    def jointTorqueCallback(self, data):
        self.jointTorques = data.data

    def jointVelocityCallback(self, data):
        self.jointVelocities = data.data

    def testParametersCallback(self, data):
        self.testParameters = data.data
        self.headingDirection    = self.testParameters[1]
        self.stability           = self.testParameters[2]
        self.distance            = self.testParameters[3]
        self.meanPower           = self.testParameters[4]
        self.meanVel             = self.testParameters[5]

    def setLegMotorPosition(self, positions):
        if self.simulationTime > 1:
            self.MotorPositionPub.publish(Float32MultiArray(data=positions))

    def terminate(self):
        return self.terminateSimulation

    def triggerSim(self):
        self.simStepDone = False

        _bool = Bool(data=True)
        self.triggerNextStepPub.publish(_bool)

        stuckTester = 0

        _bool = Bool(data=False)
        self.triggerNextStepPub.publish(_bool)

        # Wait for sim to step.
        while not self.simStepDone:
            stuckTester = stuckTester+1

            if self.simState == 0:
                return False

            # If sim does not step - then forcestep and inform
            if stuckTester > 2000000:
                rospy.logwarn("Simulation did not step.")
                self._counter = self.testParameters[0]
                return True

        # Step only one, otherwise inform
        if self._counter+1 - self.testParameters[0] > 2 and self.testParameters[0] > 1:
            rospy.logwarn("Sim overstepped: Counter is %i and expected %i", self.testParameters[0], self._counter+1)

        # Sync counter from Lua
        self._counter = self.testParameters[0]

        return True

    def synchronousSimulation(self, mode):
        _bool = Bool(data=mode)
        self.enableSyncModePub.publish(_bool)
        self._counter = 0

    def __del__(self):
        rospy.signal_shutdown('simulation ended')
