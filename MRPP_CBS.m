function [successFlag,totalCost,computeTime,PathCell] = MRPP_CBS(robotNum,MapGrid,StartRobotStates,GoalRobotStates,ctime,planTime)

successFlag=0;
computeTime=0;
totalCost=0;
PathCell=cell(robotNum,1);

%% initialize
Root.constraints=[];
solution=cell(robotNum,1);
costVec=zeros(robotNum,1);
for i=1:robotNum
    path=CBS_SinglePlanner(MapGrid,StartRobotStates(i,:),GoalRobotStates(i,:),ctime,[]);
    tempPath=path;
    sizeofpath=size(path,1);
    tempPath(sizeofpath+1:planTime,1:3)=repmat(path(end,1:3),planTime-sizeofpath,1);
    tempPath(sizeofpath+1:planTime,4)=path(end,4)+1:planTime;
    costVec(i,1)=size(path,1);
    solution{i,1}=tempPath;
end

Root.solution = solution;
Root.costVec=costVec;
Root.cost=sum(costVec);

OPEN_COUNT=1;
OPEN{OPEN_COUNT,1}=Root;
OPEN_CHECK(OPEN_COUNT)=0;%0=unckecked,1=checked

while ~isempty(find(OPEN_CHECK==0, 1))
    lowestCost=10000;
    lowestNode=0;
    indices=find(OPEN_CHECK==0);
    for i=indices'
        node=OPEN{i,1};
        if node.cost<lowestCost
            lowestNode=i;
            lowestCost=node.cost;
        end
    end
    
    OPEN_CHECK(lowestNode,1)=1;
    P=OPEN{lowestNode,1};
    
    %% conflict detection
    solution=P.solution;
    [conflictFoundFlag,constraints]=conflictDetection(solution);
    if conflictFoundFlag == 0
        PathCell=P.solution;
        totalCost=P.cost;
        break;
    end
    
    %% branching and expanding the constraint tree
    for constraintID=1:2
        agentID=constraints(constraintID,1);
        A.constraints=[P.constraints;constraints(constraintID,:)];
        pSolution=P.solution;
        AConstraints=A.constraints;
        AConstraints=AConstraints(AConstraints(:,1)==agentID,2:end);
        path=CBS_SinglePlanner(MapGrid,StartRobotStates(agentID,:),GoalRobotStates(agentID,:),ctime,AConstraints);
        %make sure robots at goal positions will stay forever
        tempPath=path;
        sizeofpath=size(path,1);
        tempPath(sizeofpath+1:planTime,1:3)=repmat(path(end,1:3),planTime-sizeofpath,1);
        tempPath(sizeofpath+1:planTime,4)=path(end,4)+1:planTime;
        pSolution{agentID,1}=tempPath;
        A.solution=pSolution;
        
        costVec=P.cost;
        costVec(agentID,1)=size(path,1);
        A.costVec=costVec;
        A.cost=sum(costVec);
        
        OPEN_COUNT=OPEN_COUNT+1;
        OPEN{OPEN_COUNT,1}=A;
        OPEN_CHECK(OPEN_COUNT,1)=0;
    end
end % while loop

end