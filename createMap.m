function [mapGrid,PodStates,PickupStationStates,DeliveryStationStates,StartStates] = createMap(width,height,rackLength,rackNumVec,aisleWidth,crossAisleWidth,topLeftRC)
mapGrid = zeros(height,width);
PickupStationGrid = zeros(height,width);

rc2xy=@(r,c,a)[c;height+1-r;a];

podNum = rackNumVec(1,1)*rackNumVec(1,2)*rackLength;
pickupStationNum = 2*(rackNumVec(1,1)-1)*rackNumVec(1,2)*rackLength;
deliveryStationNum = rackNumVec(1,1)+2;

PodStates = zeros(podNum,3);
PickupStationStates = zeros(pickupStationNum,3);
DeliveryStationStates = zeros(deliveryStationNum,3);

%% racks grid
%row indices
rows = zeros(1,rackNumVec(1,1));
for i=1:rackNumVec(1,1)
    rows(1,i) = topLeftRC(1,1)+(i-1)*(1+aisleWidth);
end
minRow = min(rows);
maxRow = max(rows);

%col indices
cols=zeros(1,rackNumVec(1,2)*rackLength);
index = 1;
for i=1:rackNumVec(1,2)
    cols(1,index:index+rackLength-1) = topLeftRC(1,2)+(i-1)*(rackLength+crossAisleWidth):topLeftRC(1,2)+(i-1)*(rackLength+crossAisleWidth)+rackLength-1;
    index = index+rackLength;
end
maxCol = max(cols);

%% generate racks, stations
for i=rows
    mapGrid(i,cols)=1;
    if i~=minRow
        PickupStationGrid(i-1,cols)=1;
    end
    if i~=maxRow
        PickupStationGrid(i+1,cols)=1;
    end
end

%pod states
[R,C]=find(mapGrid==1);
for i=1:podNum
    PodStates(i,:)=rc2xy(R(i,1),C(i,1),0);
end

%pickup station states
[R,C]=find(PickupStationGrid==1);
for i=1:size(R,1)
    if mapGrid(R(i,1)+1,C(i,1))==0 % below racks
        angle = 2;
    else
        angle = 4;
    end    
    PickupStationStates(i,:)=rc2xy(R(i,1),C(i,1),angle);
end

%delivery station states
sb=1;
for i=rows
    DeliveryStationStates(sb,:)=rc2xy(i,1,1);
    sb=sb+1;
end
for i=rows
    DeliveryStationStates(sb,:)=rc2xy(i,width,1);
    sb=sb+1;
end

% % start locations
% rows = topLeftRC(1,1):maxRow;
% cols = [topLeftRC(1,2)-6,topLeftRC(1,2)-5,topLeftRC(1,2)-3,topLeftRC(1,2)-2,maxCol+2,maxCol+3,maxCol+5,maxCol+6];
% StartGrid = zeros(height,width);
% for i=rows
%     StartGrid(i,cols)=1;
% end
% 
% StartStates = zeros(8*size(rows,1),3);
% 
% [R,C]=find(StartGrid==1);
% for i=1:size(R,1)
%     StartStates(i,:)=rc2xy(R(i,1),C(i,1),1);
% end
StartStates=[];
end
