function [conflictFoundFlag,constraints] = conflictDetection(pathCell)
%input: x-y-a-time path cell, each element is a N*4 list    
    conflictFoundFlag=0;
    constraints=zeros(2,4);
    robotNum = size(pathCell,1);
    
    for i=1:robotNum-1
        iPath=pathCell{i,1};
        iPath(:,3)=[];%delete angle information
        for j=i+1:robotNum            
            jPath=pathCell{j,1};
            jPath(:,3)=[];%delete angle information
            iLen=size(iPath,1);
            jLen=size(jPath,1);
            if iLen>1 && jLen>1
                iEdgeMat=zeros(iLen-1,6);
                iEdgeMat(:,1:3)=iPath(1:iLen-1,:);
                iEdgeMat(:,4:6)=iPath(2:iLen,:);
                jEdgeMat=zeros(jLen-1,6);
                jEdgeMat(:,1:3)=jPath(1:jLen-1,:);
                jEdgeMat(:,4:6)=jPath(2:jLen,:);
            end
            iEdgeMat(:,6)=iEdgeMat(:,3);
            iEdgeMat(:,3)=[];
            jEdgeMat(:,6)=jEdgeMat(:,3);
            jEdgeMat(:,3)=[];
            
            jReverseEdgeMat=jEdgeMat;            
            jReverseEdgeMat(:,1:2) =jEdgeMat(:,3:4);
            jReverseEdgeMat(:,3:4) =jEdgeMat(:,1:2);
            jEdgeMat=jReverseEdgeMat;  
            
            %vertex and edge conflict detection
            [vertexKind,~]=ismember(iPath,jPath,'rows');
            [edgeKind,~]=ismember(iEdgeMat,jEdgeMat,'rows');
            vertexIndices=find(vertexKind==1);
            edgeIndices=find(edgeKind==1);
            if ~isempty(vertexIndices) && ~isempty(edgeIndices) %both conflicts exist
                vertexConflict=iPath(vertexIndices(1),:);
                edgeConflict=iEdgeMat(edgeIndices(1),:);
                if vertexConflict(1,3)<edgeConflict(1,5)+1%vertex conflict happens first
                    constraints(1,:)=[i vertexConflict];
                    constraints(2,:)=[j vertexConflict];
                else
                    constraints(1,:)=[i edgeConflict(1,3:4) edgeConflict(1,5)+1];
                    constraints(2,:)=[j edgeConflict(1,1:2) edgeConflict(1,5)+1];
                end
                conflictFoundFlag=1;
                break;
                
            elseif ~isempty(vertexIndices) && isempty(edgeIndices) %only vertex conflict
                conflictFoundFlag=1;
                vertexConflict=iPath(vertexIndices(1),:);
                constraints(1,:)=[i vertexConflict];
                constraints(2,:)=[j vertexConflict];
                break;
            elseif  isempty(vertexIndices) && ~isempty(edgeIndices) %only edge conflict
                conflictFoundFlag=1;
                edgeConflict=iEdgeMat(edgeIndices(1),:);
                constraints(1,:)=[i edgeConflict(1,3:4) edgeConflict(1,5)+1];
                constraints(2,:)=[j edgeConflict(1,1:2) edgeConflict(1,5)+1];
                break;
            else
                continue;
            end
        end
        if conflictFoundFlag == 1
            break;
        end
    end
end

