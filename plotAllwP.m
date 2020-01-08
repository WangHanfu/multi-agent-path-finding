function plotAllwP(width,height,RobotStates,GoalStates,TRNum,PodStates,PickupStationStates,DeliveryStationStates,ColorMat,priorityVec)
rackMarkerSize = 95;
stationMarkerSize = 40;
robotMarkerSize = 44;
robotRadius = 0.4;
fontSize = 20;

robotNum=size(RobotStates,1);
rectangle('Position', [0,0,width+1,height+1],'lineWidth',5);

plot(PodStates(:,1),PodStates(:,2),'square','MarkerEdgeColor','k','MarkerFaceColor',[0 0 0],'MarkerSize',rackMarkerSize);
plot(PickupStationStates(:,1),PickupStationStates(:,2),'square','MarkerEdgeColor',[0.8 0.8 0.8],'MarkerFaceColor',[0.8 0.8 0.8],'MarkerSize',stationMarkerSize);
%plot(DeliveryStationStates(:,1),DeliveryStationStates(:,2),'square','MarkerEdgeColor',[0.8 0.8 0.8],'MarkerFaceColor',[0.8 0.8 0.8],'MarkerSize',stationMarkerSize+7);

for i=1:TRNum
    plot(GoalStates(i,1),GoalStates(i,2),'o','MarkerEdgeColor',ColorMat(i,:),'MarkerFaceColor',[1 1 1],'MarkerSize',robotMarkerSize,'LineWidth',3);    
end

for i=1:TRNum
    plot(RobotStates(i,1),RobotStates(i,2),'o','MarkerEdgeColor','k','MarkerFaceColor',ColorMat(i,:),'MarkerSize',robotMarkerSize);    
    txt=sprintf('%d',priorityVec(i,1));
    text(RobotStates(i,1),RobotStates(i,2),txt,'FontWeight','Bold','FontSize',fontSize,'HorizontalAlignment','center','Color',[1 1 1]);
end
plot(RobotStates(TRNum+1:robotNum,1),RobotStates(TRNum+1:robotNum,2),'o','MarkerEdgeColor','k','MarkerFaceColor',[1 1 1],'LineWidth',2,'MarkerSize',robotMarkerSize);

% %draw robot direction
% for i=1:robotNum
%     x=RobotStates(i,1);
%     y=RobotStates(i,2);
%     a=deg2rad(RobotStates(i,3));
%     xx = x+robotRadius*cos(a);
%     yy = y+robotRadius*sin(a);
%     line([x,xx],[y,yy],'color','k','linestyle','-','lineWidth',2);
% end

end

