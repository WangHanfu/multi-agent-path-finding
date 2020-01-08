clear;
clc;

robotNum=13;
instanceNumber = 50;

% StartRobotStates,GoalRobotStates,idealTotalTime
InstanceSet = cell(instanceNumber,3);

%% map parameters and create map
height=7;
width=14;
rackLength = 10;
rackNumVec = [3 1];
aisleWidth = 2;
crossAisleWidth = 1;
topLeftRC = [1 3];

[mapGrid,PodStates,PickupStationStates,DeliveryStationStates,StartStates] = createMap(width,height,rackLength,rackNumVec,aisleWidth,crossAisleWidth,topLeftRC);

%% generate instance set
for instanceCount=1:instanceNumber
    
    vec = randperm(size(PickupStationStates,1));
    TaskSet = zeros(size(PickupStationStates,1),3);
    
    for i=1:size(PickupStationStates,1)
        TaskSet(i,1:3)=PickupStationStates(vec(i),:);
    end
    
    taskIndex = 1;
    StartRobotStates = zeros(robotNum,3);
    for i=1:robotNum
        StartRobotStates(i,:)=TaskSet(taskIndex,1:3);
        taskIndex = taskIndex+1;
    end
    GoalRobotStates = zeros(robotNum,3);
    for i=1:robotNum
        GoalRobotStates(i,:)=TaskSet(taskIndex,1:3);
        taskIndex = taskIndex+1;
    end
    
    idealTotalTime = 0;
    for i=1:robotNum  
        path = singlePlanner(mapGrid,StartRobotStates(i,:),GoalRobotStates(i,:),1,[]);   
        idealTotalTime = idealTotalTime + size(path,1);
    end
    
    InstanceSet{instanceCount,1}=StartRobotStates;  
    InstanceSet{instanceCount,2}=GoalRobotStates;
    InstanceSet{instanceCount,3}=idealTotalTime; 
end

ColorMat=rand(robotNum,3);

save('instance-small-13.mat','robotNum','ColorMat','InstanceSet','mapGrid'); 