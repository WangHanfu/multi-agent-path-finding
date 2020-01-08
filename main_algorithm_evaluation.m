clear;
clc;

load('instance-small-13.mat');
instanceNum = size(InstanceSet,1);
%1 = prioritized planning
%2 = revised prioritized planning
%3 = priority-based search
%4 = revised priority-based search
%5 = enhanced priority-based search
algorithmChoose = 1;

planTime=50;
ctime = 1;

Statistics = zeros(instanceNum,3);
for instanceCount = 1:instanceNum
    disp(instanceCount);
    switch algorithmChoose
        case 1
            [successFlag,totalCost,computeTime,~] = MRPP_PP(robotNum,mapGrid,InstanceSet{instanceCount,1},InstanceSet{instanceCount,2},ctime,planTime);
        case 2
            [successFlag,totalCost,computeTime,~] = MRPP_RPP(robotNum,mapGrid,InstanceSet{instanceCount,1},InstanceSet{instanceCount,2},ctime,planTime);
        case 3
            [successFlag,totalCost,computeTime,~] = MRPP_PBS(robotNum,mapGrid,InstanceSet{instanceCount,1},InstanceSet{instanceCount,2},ctime,planTime);
        case 4
            [successFlag,totalCost,computeTime,~] = MRPP_RPBS(robotNum,mapGrid,InstanceSet{instanceCount,1},InstanceSet{instanceCount,2},ctime,planTime);
        case 5
            [successFlag,totalCost,computeTime,~] = MRPP_EPBS(robotNum,mapGrid,InstanceSet{instanceCount,1},InstanceSet{instanceCount,2},ctime,planTime);
    end
    Statistics(instanceCount,:)=[successFlag totalCost computeTime];
end

successIndices = find(Statistics(:,1)==1);
prolongationSum = 0;
computeTimeSum = 0;
for i = successIndices'
    sum1 = Statistics(i,2);
    idealTimes = InstanceSet{i,3};
    prolongationSum=prolongationSum+(sum1-idealTimes)/idealTimes;
    computeTimeSum=computeTimeSum+Statistics(i,3);
end

coverageRate = size(successIndices,1)/instanceNum;
prolongation = prolongationSum/size(successIndices,1);
aveComputeTime = computeTimeSum/size(successIndices,1);
fprintf('coverage=%f,prologation=%f,aveComputeTime=%f',coverageRate,prolongation,aveComputeTime);
