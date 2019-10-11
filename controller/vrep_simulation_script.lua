-- Global functions and Constants
local socket = require 'socket'
math.randomseed(os.time())

local function roundToNthDecimal(num, n)
    local mult = 10^(n or 0)
    return math.floor(num * mult + 0.5) / mult
end

function mean( t )
    local sum = 0
    local count= 0
    for k,v in pairs(t) do
        if type(v) == 'number' then
            sum = sum + v
            count = count + 1
        end
    end
    return (sum / count)
end

function standardDeviation( t )
    local m
    local vm
    local sum = 0
    local count = 0
    m = mean( t )
    for k,v in pairs(t) do
        if type(v) == 'number' then
            vm = v - m
            sum = sum + (vm * vm)
            count = count + 1
        end
    end
    return math.sqrt(sum / (count-1))
end

function simGetJointVelocity (jointHandle)
    res,velocity=simGetObjectFloatParameter(jointHandle,2012)
    return  velocity
end

function absmean (prev_avg, x, n)
    return ((prev_avg * n + math.abs(x)) / (n + 1));
end

function absmean_arr (numlist)
    if type(numlist) ~= 'table' then
        print("Error in absmean_arr")
        return numlist
    end
    num = 0
    table.foreach(numlist,function(i,v) num=num+math.abs(v) end)
    return num / #numlist
end

-- Set motor positions callback function
function setMotorPositions_cb(msg)
    data = msg.data
    sim.setJointTargetPosition(TC_motor0,data[1])
    sim.setJointTargetPosition(CF_motor0,data[2])
    sim.setJointTargetPosition(TC_motor1,data[3])
    sim.setJointTargetPosition(CF_motor1,data[4])
    sim.setJointTargetPosition(TC_motor2,data[5])
    sim.setJointTargetPosition(CF_motor2,data[6])
    sim.setJointTargetPosition(TC_motor3,data[7])
    sim.setJointTargetPosition(CF_motor3,data[8])
end

--[[
Initialization: Called once at the start of a simulation
--]]
if (sim_call_type==sim.childscriptcall_initialization) then
    print(' ')
    print("****************")
    print(" MAT CRAWLER")
    print("****************")

    stepCounter = 0;
    mean_vel = 0;
    mean_jtor = 0;
    mean_jvel = 0;
    mean_jpower = 0;
    update_count = 0;
    height_arr = {}
    oriX_arr = {}
    oriY_arr = {}
    orientation_arr = {}
    testParameters = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

    -- Create all handles
    robotHandle=sim.getObjectAssociatedWithScript(sim.handle_self)

    TC_motor0=sim.getObjectHandle("TC1")    -- Handle of the TC motor
    TC_motor1=sim.getObjectHandle("TC2")    -- Handle of the TC motor
    TC_motor2=sim.getObjectHandle("TC3")    -- Handle of the TC motor
    TC_motor3=sim.getObjectHandle("TC4")    -- Handle of the TC motor

    CF_motor0=sim.getObjectHandle("CF1")    -- Handle of the CF motor
    CF_motor1=sim.getObjectHandle("CF2")    -- Handle of the CF motor
    CF_motor2=sim.getObjectHandle("CF3")    -- Handle of the CF motor
    CF_motor3=sim.getObjectHandle("CF4")    -- Handle of the CF motor

    IMU=sim.getObjectHandle("Imu")
    matcrawlerHandle=sim.getObjectHandle("matcrawler")

    -- Check if the required ROS plugin is loaded
    moduleName=0
    moduleVersion=0
    index=0
    pluginNotFound=true
    while moduleName do
        moduleName,moduleVersion=sim.getModuleName(index)
        if (moduleName=='RosInterface') then
            pluginNotFound=false
        end
        index=index+1
    end
    if (pluginNotFound) then
        sim.displayDialog('Error','The RosInterface was not found.',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
        printToConsole('[ERROR] The RosInterface was not found.')
    end

    -- If found then start the subscribers and publishers
    if (not pluginNotFound) then
        local simulationTimeTopicName='sim_control/simulationTime'
        local startSimulationName='sim_control/startSimulation'
        local pauseSimulationName='sim_control/pauseSimulation'
        local stopSimulationName='sim_control/stopSimulation'
        local enableSyncModeName='sim_control/enableSyncMode'
        local triggerNextStepName='sim_control/triggerNextStep'
        local simulationStepDoneName='sim_control/simulationStepDone'
        local simulationStateName='sim_control/simulationState'

        local terminateControllerName='sim_control/terminateController'
        local plotterName='sim_control/plotter'
        local MotorTopicName='morf_sim/multi_joint_command'
        local jointPositionsName='morf_sim/joint_positions'
        local jointTorquesName='morf_sim/joint_torques'
        local jointVelocitiesName='morf_sim/joint_velocities'
        local testParametersName='sim_control/testParameters'

        -- Create the subscribers
        MotorSub=simROS.subscribe('/'..MotorTopicName,'std_msgs/Float32MultiArray','setMotorPositions_cb')
        PlotterSub=simROS.subscribe('/'..plotterName,'std_msgs/Float32MultiArray', 'graph_cb')

        -- Create the publishers
        terminateControllerPub=simROS.advertise('/'..terminateControllerName,'std_msgs/Bool')
        jointPositionsPub=simROS.advertise('/'..jointPositionsName,'std_msgs/Float32MultiArray')
        jointTorquesPub=simROS.advertise('/'..jointTorquesName,'std_msgs/Float32MultiArray')
        jointVelocitiesPub=simROS.advertise('/'..jointVelocitiesName,'std_msgs/Float32MultiArray')
        imuEulerPub=simROS.advertise('/morf_sim/euler','geometry_msgs/Vector3')
        testParametersPub=simROS.advertise('/'..testParametersName,'std_msgs/Float32MultiArray')
        clockPub=simROS.advertise('/clock','rosgraph_msgs/Clock')

        -- Start the client application (python node)
        whereismycake =  ""
        os.execute("python ./../../controller/main.py" .." "..MotorTopicName.." "..simulationTimeTopicName.." "..terminateControllerName.." "..startSimulationName.." "..pauseSimulationName.." "..stopSimulationName.." "..enableSyncModeName.." "..triggerNextStepName.." "..simulationStepDoneName.." "..simulationStateName.." "..plotterName.." "..jointPositionsName.." "..jointTorquesName.." "..jointVelocitiesName.." "..testParametersName.." &")
        if (result==false) then
            sim.displayDialog('Error','Python controller not found.',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
            printToConsole('[ ERROR] Python controller not found')
        else
            print('[ INFO] Python Controller started')
        end
    end

    -- Init position to zero
    sim.setJointTargetPosition(TC_motor0,0)
    sim.setJointTargetPosition(CF_motor0,0)
    sim.setJointTargetPosition(TC_motor1,0)
    sim.setJointTargetPosition(CF_motor1,0)
    sim.setJointTargetPosition(TC_motor2,0)
    sim.setJointTargetPosition(CF_motor2,0)
    sim.setJointTargetPosition(TC_motor3,0)
    sim.setJointTargetPosition(CF_motor3,0)

    -- WAIT FOR C++ CONTROLLER TO LAUNCH
    socket.sleep(0.1)

    simROS.publish(terminateControllerPub,{data=false})
    printToConsole('[ INFO] Initialized simulation')
end

--[[
Actuation: This part will be executed in each simulation step
--]]
if (sim_call_type==sim.childscriptcall_actuation) then
    -- Publish
    simROS.publish(terminateControllerPub,{data=false})
    simROS.publish(clockPub,{clock=simGetSimulationTime()})
end

--[[
Sensing: This part will be executed in each simulation step
--]]
if (sim_call_type==sim.childscriptcall_sensing) then
    -- JOINT DATA
    position_array  ={  simGetJointPosition(TC_motor0),simGetJointPosition(CF_motor0),
                        simGetJointPosition(TC_motor1),simGetJointPosition(CF_motor1),
                        simGetJointPosition(TC_motor2),simGetJointPosition(CF_motor2),
                        simGetJointPosition(TC_motor3),simGetJointPosition(CF_motor3) }

    velocity_array  ={  simGetJointVelocity(TC_motor0),simGetJointVelocity(CF_motor0),
                        simGetJointVelocity(TC_motor1),simGetJointVelocity(CF_motor1),
                        simGetJointVelocity(TC_motor2),simGetJointVelocity(CF_motor2),
                        simGetJointVelocity(TC_motor3),simGetJointVelocity(CF_motor3) }

    torque_array    ={  simGetJointForce(TC_motor0),simGetJointForce(CF_motor0),
                        simGetJointForce(TC_motor1),simGetJointForce(CF_motor1),
                        simGetJointForce(TC_motor2),simGetJointForce(CF_motor2),
                        simGetJointForce(TC_motor3),simGetJointForce(CF_motor3) }
    -- IMU DATA
    linearVelocity, aVelocity=sim.getObjectVelocity(IMU) -- m/s
    imuPosition = sim.getObjectPosition(IMU,-1)
    imuOrientation = sim.getObjectOrientation(IMU,-1)
    table.insert(oriX_arr, imuOrientation[1])
    table.insert(oriY_arr, imuOrientation[2])
    table.insert(orientation_arr, imuOrientation[3]) -- Walking direction array

    -- Mean velocity of the robot
    mean_vel = absmean(mean_vel, linearVelocity[1], update_count)

    -- Mean power of the robot
    mean_jtor   = absmean(mean_jtor, absmean_arr(torque_array), update_count)
    mean_jvel   = absmean(mean_jvel, absmean_arr(velocity_array), update_count)
    mean_jpower = absmean(mean_jpower, mean_jtor * mean_jvel, update_count)

    -- Distance walked
    positionRobot=sim.getObjectPosition(matcrawlerHandle, -1)
    distance = -positionRobot[2] -- use negative world y axis

    -- Collect into test parameter array
    testParameters[2]=math.abs(imuOrientation[3])
    testParameters[3]=(standardDeviation(oriX_arr)+standardDeviation(oriY_arr))/2 -- Stability
    testParameters[4]=distance
    testParameters[5]=mean_jpower
    testParameters[6]=mean_vel

    -- Step counter for sync with controller
    stepCounter = stepCounter + 1
    testParameters[1] = stepCounter
    update_count = update_count + 1

    -- Publish all sensor data
    simROS.publish(jointPositionsPub,{data=position_array})
    simROS.publish(jointVelocitiesPub,{data=velocity_array})
    simROS.publish(jointTorquesPub,{data=torque_array})
    simROS.publish(testParametersPub,{data=testParameters})
end

--[[
Clean up: This part will be executed one time just before a simulation ends
--]]
if (sim_call_type==sim.childscriptcall_cleanup) then

    print(" ")
    print("Direction stdv:\t" .. roundToNthDecimal(standardDeviation(orientation_arr),4))
    print("Stability stdv:\t" .. roundToNthDecimal(((standardDeviation(oriX_arr)+standardDeviation(oriY_arr))/2),4))
    print("Distance:\t" .. roundToNthDecimal(distance,4))
    print("Mean power:\t" .. roundToNthDecimal(mean_jpower,4))
    print("Mean velocity:\t" .. roundToNthDecimal(mean_vel,4))
    print(" ")

    print('[INFO] terminating')
    simROS.publish(terminateControllerPub,{data=true})

    -- Wait for the signal to reach the node
    waitTimer=0
    while( waitTimer < 1000 ) do
        waitTimer = waitTimer+1
        simROS.publish(clockPub,{clock=simGetSimulationTime()+waitTimer})
        simROS.publish(terminateControllerPub,{data=true})
    end

    -- Terminate remaining local notes
    simROS.shutdownSubscriber(PlotterSub)
    simROS.shutdownSubscriber(MotorSub)
    simROS.shutdownPublisher(clockPub)
    simROS.shutdownPublisher(jointTorquesPub)
    simROS.shutdownPublisher(jointVelocitiesPub)
    simROS.shutdownPublisher(jointPositionsPub)
    simROS.shutdownPublisher(testParametersPub)
    simROS.shutdownPublisher(terminateControllerPub)
    simROS.shutdownPublisher(imuEulerPub)

    printToConsole('[ INFO] Lua child script stopped')
end
