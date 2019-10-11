#!/usr/bin/env python

import sys
import time
import numpy as np
import simROS
import cpg

def rescale(newMax, newMin, parameter):
    return (((newMax-newMin) * (parameter+0.2)) / (0.2+0.2)) + newMin


def main(argv):
    # Init ROS communication
    ros_handle = simROS.SIMROS(argv)
    time.sleep(0.2)  # wait for connections to form
    ros_handle.synchronousSimulation(True)

    # Init CPG
    cpg_handle = cpg.CPG(0.05*np.pi)

    # Run controller
    while True:
        # If terminate signal is true then exit program
        if ros_handle.terminate():
            print('[ INFO Python main script stopped]')
            ros_handle.synchronousSimulation(False)
            # wait for msg to send
            time.sleep(0.1)
            del ros_handle
            sys.exit(0)

        # Get CPG output
        cpg0 = cpg_handle.get_output(0)
        cpg1 = cpg_handle.get_output(1)

        # Get feedback and sensory information from simulation


        # Rescale CPG output to fit robot
        motor_positions = [rescale(0.4, -0.4, cpg1), #TC1
                           rescale(0.0, -0.6, cpg0), #CF1
                           rescale(0.4, -0.4, cpg1), #TC2
                           rescale(0.0, -0.6, cpg0), #CF2
                           rescale(0.4, -0.4, cpg1), #TC3
                           rescale(0.0, -0.6, cpg0), #CF3
                           rescale(0.4, -0.4, cpg1), #TC4
                           rescale(0.0, -0.6, cpg0)] #CF4

        # Send rescaled CPG output to the motors
        ros_handle.setLegMotorPosition(motor_positions)

        # Step the CPG
        cpg_handle.step()

        # Trigger one time step (50ms) in simulation
        ros_handle.triggerSim()


if __name__ == '__main__':
    main(sys.argv[1:])
