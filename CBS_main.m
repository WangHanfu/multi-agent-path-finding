clear;
clc;

%% map parameters and create map
height=7;
width=14;
rackLength = 10;
rackNumVec = [3 1];
aisleWidth = 2;
crossAisleWidth = 1;
topLeftRC = [1 3];

[mapGrid,PodStates,PickupStationStates,DeliveryStationStates,StartStates] = createMap(width,height,rackLength,rackNumVec,aisleWidth,crossAisleWidth,topLeftRC);

%% robots
robotNum=10;
TRNum=robotNum;
RobotCell = cell(robotNum,1);

%% Task set
taskIndex = 1;
vec = randperm(size(PickupStationStates,1));
TaskSet = zeros(size(PickupStationStates,1),4);
for i=1:size(PickupStationStates,1)
    TaskSet(i,1:3)=PickupStationStates(vec(i),:);
    TaskSet(i,4)=randi([5 15]);
end

for i=1:robotNum
    RobotCell{i,1}=Robot;
    if i<=TRNum
        type = 0;
    else
        type = 1;
    end
    RobotCell{i,1}.setAttribute(i,type,TaskSet(taskIndex,1:3));
    taskIndex = taskIndex+1;
end

sz=get(0,'screensize');
sz(1,2) = 10;
sz(1,4) = 1080;
h=figure('outerposition',sz);
assignin('base','h',h); %in case of any callback errors.
hold on;
grid on;
set(gca,'xtick',0:1:width);
set(gca,'ytick',0:1:height);
axis equal;
axis([0 width+1 0 height+1]);
axis manual;

StartRobotStates = zeros(robotNum,3);
for i=1:robotNum
    StartRobotStates(i,:)=RobotCell{i,1}.getState;
end

GoalRobotStates = zeros(robotNum,3);
for i=1:robotNum
    GoalRobotStates(i,:)=TaskSet(taskIndex,1:3);
    taskIndex = taskIndex+1;
end

ColorMat=rand(robotNum,3);

save('example.mat','ColorMat','StartRobotStates','GoalRobotStates');

load('example.mat');

plotAll(width,height,StartRobotStates,GoalRobotStates,robotNum,PodStates,PickupStationStates,DeliveryStationStates,ColorMat);

%% system evolution
simTime = 1;
planTime=30;
nextRobotStates = zeros(robotNum,3);
currentRobotTaskStatus = zeros(robotNum,1);

%[~,~,~,AllPathCell] = MRPP_PP(robotNum,mapGrid,StartRobotStates,GoalRobotStates,simTime,planTime);
[~,~,~,AllPathCell] = MRPP_CBS(robotNum,mapGrid,StartRobotStates,GoalRobotStates,simTime,planTime);

intepolate = 10;
video = VideoWriter('simulation');
video.FrameRate=intepolate;
open(video);
while(simTime<planTime)
    for i=1:robotNum
        path = AllPathCell{i,1};
        StartRobotStates(i,:)=path(simTime,1:3);     
        nextRobotStates(i,:)=path(simTime+1,1:3);
    end            
    %continuous simulation
    for i=0:intepolate-1
        tempRobotStates=StartRobotStates+i*(nextRobotStates-StartRobotStates)/intepolate;
        currentAngles=StartRobotStates(:,3)*90-90;
        nextAngles=nextRobotStates(:,3)*90-90;
        flag1 = nextRobotStates(:,3)-StartRobotStates(:,3)==3; %right to down
        flag2 = nextRobotStates(:,3)-StartRobotStates(:,3)==-3; % down to right
        nextAngles = nextAngles+360*flag2-360*flag1;
        tempRobotStates(:,3)=currentAngles+i*(nextAngles-currentAngles)/intepolate;
        plotAll(width,height,tempRobotStates,GoalRobotStates,TRNum,PodStates,PickupStationStates,DeliveryStationStates,ColorMat);
        frame = getframe;
        writeVideo(video,frame);
        cla;
    end    
    simTime = simTime+1;
end

close(video);