robotNum=2;
MapGrid=zeros(2,5);
MapGrid(1,1)=1;
MapGrid(1,2)=1;
MapGrid(1,4)=1;
MapGrid(1,5)=1;
StartRobotStates=[1 1 1;5 1 1];
GoalRobotStates=[4 1 1;2 1 1];
ctime=1;
planTime=10;

[successFlag,totalCost,computeTime,PathCell] = MRPP_CBS(robotNum,MapGrid,StartRobotStates,GoalRobotStates,ctime,planTime);

% pathCell=cell(2,1);
% pathCell{1,1}=[1 1 0 0; 1 1 0 1;2 1 0 2];
% pathCell{2,1}=[2 1 0 1;1 1 0 2;2 1 0 2];
% 
% [conflictFoundFlag,constraints]=conflictDetection(pathCell);