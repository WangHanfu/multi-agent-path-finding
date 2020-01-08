function [successFlag,totalCost,computeTime,AllPathCell] = MRPP_RPP(robotNum,mapGrid,StartRobotStates,GoalRobotStates,ctime,planTime)
    
    successFlag=1;
    computeTime=0;
    totalCost=0;
    
    % reserved temporal-spatial table
    currentTSTable = [];
    
    AllPathCell=cell(robotNum,1);    
    currentTSTable=[];
    t1=clock;
    for i=1:robotNum
        tempTSTable = [];
        %avoid higher priority robots' goal stations and lower priority
        %robots' start stations
        for j=1:robotNum
            sb = zeros(planTime,3);
            if i<j
                sb(:,1:2)=repmat(StartRobotStates(j,1:2),planTime,1);
                sb(:,3)=(1:planTime)';
                tempTSTable = [tempTSTable;sb];
            elseif i>j
                sb(:,1:2)=repmat(GoalRobotStates(j,1:2),planTime,1);
                sb(:,3)=(1:planTime)';
                tempTSTable = [tempTSTable;sb];
            else
                continue;
            end
        end
        tempTSTable = [tempTSTable;currentTSTable];
        tempPath=singlePlannerForPP(mapGrid,StartRobotStates(i,:),GoalRobotStates(i,:),ctime,tempTSTable);
        if size(tempPath,1)==1 || size(tempPath,1)>=planTime
            successFlag=0;
            break;
        end
        pathLength=size(tempPath,1);
        totalCost=totalCost+pathLength;
        sb=tempPath(end,:);   
        %makesure robots stays forever on the goal station.
        for j=1:planTime-pathLength
            tempPath(pathLength+j,:)=[sb(1,1:3),j+pathLength];
        end
        AllPathCell{i,1}=tempPath;
        tempPath(:,3)=tempPath(:,4);
        tempPath(:,4)=[];
        currentTSTable=[currentTSTable;tempPath];    
    end
    t2=clock;
    computeTime=etime(t2,t1);
    if successFlag == 0
        computeTime = 0;
        totalCost = 0;
        AllPathCell=[];
    end    
end

