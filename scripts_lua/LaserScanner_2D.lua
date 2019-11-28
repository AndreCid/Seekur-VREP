function sysCall_init() 
    laserHandle=sim.getObjectHandle("LaserScannerLaser_2D")
    jointHandle=sim.getObjectHandle("LaserScannerJoint_2D")
    graphHandle=sim.getObjectHandle("LaserScannerGraph_2D")
    modelHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    objName=sim.getObjectName(modelHandle)
    communicationTube=sim.tubeOpen(0,objName..'_2D_SCANNER_DATA',1)
    handler_laser = sim.getObjectHandle("LaserScanner_2D")

    -- Laser publisher using LaserScan msg as http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html 
    pub_scan = simROS.advertise('/scan','sensor_msgs/LaserScan')
    

    -- Some info abaout the laser to pub on ROS

    mim_angle = 0 -- start angle of the scan [rad]
    max_angle = 180 -- end angle of the scan [rad]
    angle_increment = 0.0001 -- angular distance between measurements [rad]
    time_increment = 0.01 -- time between measurements [seconds]
    range_min = 0.01 -- minimum range value [m]       
    range_max = 15 -- maximum range value [m]  
    scanAngle = 180 --laser's angle 

end

function sysCall_cleanup() 
    sim.resetGraph(graphHandle)
end 

function sysCall_sensing() 

    points={}
    laser_xData = {}
    laser_yData = {}
    laser_data = {}
    time_current=sim.getSimulationTime()

    scanningAngle=tonumber(sim.getScriptSimulationParameter(sim.handle_self,"scanningAngle"))
    if (scanningAngle==nil) then
        scanningAngle=180
        sim.setScriptSimulationParameter(sim.handle_self,"scanningAngle",scanningAngle)
    end
    if (scanningAngle<5) then
        scanningAngle=5
    end
    if (scanningAngle>180) then
        scanningAngle=180
    end
    scanningAngle = 180
    scanningDensity=tonumber(sim.getScriptSimulationParameter(sim.handle_self,"scanningDensity"))
    if (scanningDensity==nil) then
        scanningDensity=2
        sim.setScriptSimulationParameter(sim.handle_self,"scanningDensity",scanningDensity)
    end
    if (scanningDensity<0.1) then
        scanningDensity=0.1
    end
    if (scanningDensity>5) then
        scanningDensity=5
    end
    
    sim.resetGraph(graphHandle)
    pts=scanningAngle*scanningDensity+1
    p=-scanningAngle*math.pi/360
    stepSize=math.pi/(scanningDensity*180)
    points={}
    modelInverseMatrix=simGetInvertedMatrix(sim.getObjectMatrix(modelHandle,-1))
    for i=0,pts,1 do
        sim.setJointPosition(jointHandle,p)
        p=p+stepSize
        r,dist,pt=sim.handleProximitySensor(laserHandle) -- pt is RELATIVE to te rotating laser beam!
        if r>0 then
        -- We put the RELATIVE coordinate of that point into the table that we will return:
            m=sim.getObjectMatrix(laserHandle,-1)
            pt=sim.multiplyVector(m,pt)
            pt=sim.multiplyVector(modelInverseMatrix,pt) -- after this instruction, pt will be relative to the model base!
            table.insert(points,pt[1])
            table.insert(points,pt[2])
            table.insert(points,pt[3])
        else
            table.insert(points,20)
            table.insert(points,0)
            table.insert(points,0)
        end
        sim.handleGraph(graphHandle,0.0)
    end
    -- Adapting the measuredData message to RVIZ
    t = table.getn(points)
 
    for i=2,t,3 do
        table.insert(laser_xData, points[i])
    end
    for i=3,t,3 do
        table.insert(laser_yData, points[i])
    end

    for i=1,table.getn(laser_xData),1 do
        dist_laser = math.sqrt((laser_yData[i]^2)+(laser_xData[i]^2))
        table.insert(laser_data,dist_laser)
    end

    -- Ajust angle increment 
    angle_increment =  math.rad(scanningAngle)/table.getn(laser_data)
    

    pub_scan_message={}
    pub_scan_message["header"]={stamp=time_current,frame_id="laser_scan"}
    pub_scan_message["angle_min"]=mim_angle
    pub_scan_message["angle_max"]=max_angle
    pub_scan_message["angle_increment"]=angle_increment
    pub_scan_message["time_increment"]=time_increment
    pub_scan_message["scan_time"]=time_current
    pub_scan_message["range_min"]=range_min
    pub_scan_message["range_max"]=range_max
    pub_scan_message["ranges"]=laser_data
    pub_scan_message["intensities"]={}
   

    simROS.publish(pub_scan,pub_scan_message)

    
    --simROS.publish(pub_scan,pub_scan_message)

-- Now send the data:
    if #points>0 then
        sim.tubeWrite(communicationTube,sim.packFloatTable(points))
    end
    
    -- To read the data from another script, use following instructions (in that other script):
    --
    -- INITIALIZATION PHASE:
    -- laserScannerHandle=sim.getObjectHandle("LaserScanner_2D")
    -- laserScannerObjectName=sim.getObjectName(laserScannerHandle) -- is not necessarily "LaserScanner_2D"!!!
    -- communicationTube=sim.tubeOpen(0,laserScannerObjectName..'_2D_SCANNER_DATA',1)
    --
    -- TO READ THE DATA:
    -- data=sim.tubeRead(communicationTube)
    -- if (data) then
    --     laserDetectedPoints=sim.unpackFloatTable(data)
    -- end
    --
    -- laserDetectedPoints is RELATIVE to the model base!
end 

