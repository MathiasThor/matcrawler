---
--- Created by mat.
--- DateTime: 5/22/19 1:14 PM
---

local socket = require 'socket'

function sysCall_init()
    rosInterfaceSynModeEnabled=false
    haltMainScript=false

    -- Check if the RosInterface is available:
    moduleName=0
    moduleVersion=0
    index=0
    pluginFound=false
    while moduleName do
        moduleName,moduleVersion=sim.getModuleName(index)
        if (moduleName=='RosInterface') then
            pluginFound=true
        end
        index=index+1
    end

    if pluginFound then
        startSub=simROS.subscribe('/sim_control/startSimulation', 'std_msgs/Int32', 'startSimulation_callback')
        pauseSub=simROS.subscribe('/sim_control/pauseSimulation', 'std_msgs/Int32', 'pauseSimulation_callback')
        stopSub=simROS.subscribe('/sim_control/stopSimulation', 'std_msgs/Bool', 'stopSimulation_callback')
        enableSynModeSub=simROS.subscribe('/sim_control/enableSyncMode', 'std_msgs/Bool', 'enableSyncMode_callback')
        triggerNextStepSub=simROS.subscribe('/sim_control/triggerNextStep', 'std_msgs/Bool', 'triggerNextStep_callback')
        blackoutSub=simROS.subscribe('/sim_control/blackout', 'std_msgs/Bool', 'blackout_callback')

        simStepDonePub=simROS.advertise('/sim_control/simulationStepDone', 'std_msgs/Bool')
        simStatePub=simROS.advertise('/sim_control/simulationState','std_msgs/Int32')
        simTimePub=simROS.advertise('/sim_control/simulationTime','std_msgs/Float32')
        auxPub=simROS.advertise('/sim_control/privateMsgAux', 'std_msgs/Bool')
        auxSub=simROS.subscribe('/sim_control/privateMsgAux', 'std_msgs/Bool', 'aux_callback')
    else
        sim.displayDialog('Error','The RosInterface was not found.',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
    end
end

function startSimulation_callback(msg)
    sim.startSimulation()
end

function blackout_callback(msg)
    data=msg.data
    sim.setBoolParameter(sim.boolparam_display_enabled,data)
end

function pauseSimulation_callback(msg)
    sim.stopSimulation()
    socket.sleep(0.3)
    result=sim.setIntegerSignal("rollout",msg.data)
    sim.startSimulation()
end

function stopSimulation_callback(msg)
    sim.stopSimulation()
end

function enableSyncMode_callback(msg)
    rosInterfaceSynModeEnabled=msg.data
    haltMainScript=rosInterfaceSynModeEnabled

    if rosInterfaceSynModeEnabled then
        print("[ INFO] SyncMode enabled")
    else
        print("[ INFO] SyncMode disabled")
    end

end

function triggerNextStep_callback(msg)
    haltMainScript=false
end

function aux_callback(msg)
    simROS.publish(simStepDonePub,{data=true})
end

function publishSimState()
    local state=0 -- simulation not running
    local s=sim.getSimulationState()
    if s==sim.simulation_paused then
        state=2 -- simulation paused
    elseif s==sim.simulation_stopped then
        state=0 -- simulation stopped
    else
        state=1 -- simulation running
    end
    simROS.publish(simStatePub,{data=state})
end


function sysCall_nonSimulation()
    if pluginFound then
        publishSimState()
    end
end

function sysCall_beforeMainScript()
    return {doNotRunMainScript=haltMainScript}
end

function sysCall_actuation()
    if pluginFound then
        publishSimState()
        simROS.publish(simTimePub,{data=sim.getSimulationTime()})
    end
end

function sysCall_sensing()
    if pluginFound then
        simROS.publish(auxPub,{data=true})
        haltMainScript=rosInterfaceSynModeEnabled
    end
end

function sysCall_suspended()
    if pluginFound then
        publishSimState()
    end
end

function sysCall_afterSimulation()
    if pluginFound then
        publishSimState()
    end
end

function sysCall_cleanup()
    if pluginFound then
        simROS.shutdownSubscriber(startSub)
        simROS.shutdownSubscriber(pauseSub)
        simROS.shutdownSubscriber(stopSub)
        simROS.shutdownSubscriber(enableSynModeSub)
        simROS.shutdownSubscriber(triggerNextStepSub)
        simROS.shutdownSubscriber(blackoutSub)
        simROS.shutdownSubscriber(auxSub)
        simROS.shutdownPublisher(auxPub)
        simROS.shutdownPublisher(simStepDonePub)
        simROS.shutdownPublisher(simStatePub)
        simROS.shutdownPublisher(simTimePub)
    end
end
