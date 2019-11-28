

function sysCall_init()
   
    -- Pegando o handler do robo
    robotHandle = sim.getObjectAssociatedWithScript(sim.handle_self)
    handler_laser = sim.getObjectHandle("LaserScanner_2D")

    -- Pegando o handler dos motores
    mot = {}
    for i=1,4 do
        mot[i] = sim.getObjectHandle("motor"..i)
    end

    -- Iniciando a API de comunicacao com o MATLAB
     simRemoteApi.start(19999)

    -- Inicializando a conexao com o ROS
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

    -- chamando inicializacao
    if (not pluginNotFound) then
        local sysTime=sim.getSystemTime() 
        local MotorSpeedTopicName='velMotores' -- we add a random component so that we can have several instances of this robot running
       
        pubPosicao = simROS.advertise('/pose','geometry_msgs/Pose2D')
        
        ros_clock = simROS.advertise('/clock','rosgraph_msgs/Clock')
        sub_vel=simROS.subscribe('/cmd_vel', 'geometry_msgs/Twist', 'cmd_vel__callback')
        
    end


    print("=== Inicializacao pronta. Iniciando atuacao ===")
    
    InitialPose = getTransformStamped(robotHandle ,'static_InitialPose',-1,'map')
    base_link = getTransformStamped(robotHandle ,'static_InitialPose',-1,'map')
    simROS.sendTransform(InitialPose)

    wheel_radius = 0.2
    wheel_separation = 0.30
end

function sysCall_actuation()

    -- Obtem a posicao do robo
        getPos = {}
        getOri = {}
    	getPos = sim.getObjectPosition(robotHandle, -1)
        getOri = sim.getObjectOrientation(robotHandle, -1)

       

    -- Publica a posicao do robo
        tabela_pos = {}
        tabela_pos["x"] = getPos[1]
        tabela_pos["y"] = getPos[2]
        --tabela_pos["z"] = getPos[3]
        tabela_pos["theta"] = getOri[3]
      
        simROS.publish(ros_clock,{clock=sim.getSimulationTime()})
        simROS.publish(pubPosicao, tabela_pos)
        base_link = getTransformStamped(robotHandle,"base_link",-1,"map")
        sendTransformCurrentTime(base_link)
        base_laser = getTransformStamped(handler_laser,"laser_scan",-1,"base_link")
        sendTransformCurrentTime(base_laser)
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- Callback do ROS
function cmd_vel__callback(msg)
    
    speed_left = (msg['linear']['x'] + msg['angular']['z'] * wheel_separation/2)/wheel_radius
    speed_right = (msg['linear']['x'] - msg['angular']['z'] * wheel_separation/2)/wheel_radius
        
    
    sim.setJointTargetVelocity(mot[3],speed_left)
    sim.setJointTargetVelocity(mot[4],speed_left)
    sim.setJointTargetVelocity(mot[1],speed_right)
    sim.setJointTargetVelocity(mot[2],speed_right)

end

function getTransformStamped(objHandle,name,relTo,relToName)

    -- obtain the body pose
    t=sim.getSimulationTime()
    p=sim.getObjectPosition(objHandle,relTo)
    o=sim.getObjectQuaternion(objHandle,relTo)
    
    -- mounting the transform
    return{
        header={
            stamp=t,
            frame_id=relToName
        },
        child_frame_id=name,
        transform={
            -- ROS has definition x=front y=side z=up
            translation={x=p[1],y=p[2],z=p[3]},--V-rep
            rotation={x=o[1],y=o[2],z=o[3],w=o[4]}--v-rep
        }
    }
end

function sendTransformCurrentTime(tf_input)
    
    -- receives the current time
    local t = sim.getSimulationTime()
    
    local tf_output = {
        header={
            stamp=t,
            frame_id=tf_input["header"]["frame_id"]
        },
        child_frame_id=tf_input["child_frame_id"],
        transform=tf_input["transform"]
    }

    simROS.sendTransform(tf_output)
end

-- You can define additional system calls here:
--[[
function sysCall_suspend()
end

function sysCall_resume()
end

function sysCall_jointCallback(inData)
    return outData
end

function sysCall_contactCallback(inData)
    return outData
end

function sysCall_beforeCopy(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." will be copied")
    end
end

function sysCall_afterCopy(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." was copied")
    end
end

function sysCall_beforeDelete(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." will be deleted")
    end
    -- inData.allObjects indicates if all objects in the scene will be deleted
end

function sysCall_afterDelete(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." was deleted")
    end
    -- inData.allObjects indicates if all objects in the scene were deleted
end
--]]
