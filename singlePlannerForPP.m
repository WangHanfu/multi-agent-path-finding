function path = singlePlannerForPP(mapGrid,startState,goalState,ctime,constraints)
    [height,~]=size(mapGrid);
    xy2rc=@(x,y,a)[height+1-y;x;a];
    startRCA(1,1:3)=xy2rc(startState(1,1),startState(1,2),startState(1,3));
    goalRCA(1,1:3)=xy2rc(goalState(1,1),goalState(1,2),goalState(1,3));
    tempConstraints = constraints;
    maxTime=0;
    if size(tempConstraints,1)~=0
        tempConstraints(:,1)=height+1-constraints(:,2);
        tempConstraints(:,2)=constraints(:,1);
        tempConstraints(:,3)=tempConstraints(:,3)-ctime;
        %find future occupied temporal-spatial locations by higher priority
        %robots
        vec = ismember(tempConstraints(:,1:2),goalRCA(1,1:2),'rows');
        indices = find(vec==1);
        sb = tempConstraints(indices',:);
        maxTime = max(sb(:,3));
        if isempty(maxTime)
            maxTime=0;
        end
    end
        
    %AStartSTT start from time=0
    tempPath=AStarSTT(mapGrid,startRCA(1,1:2),goalRCA(1,1:2),tempConstraints,maxTime);
    path=tempPath;
    path(:,1)=tempPath(:,2);
    path(:,2)=height+1-tempPath(:,1);
    path(:,3)=path(:,3)+ctime;
    path(:,4)=path(:,3);
    path(:,3)=0;
        
end


