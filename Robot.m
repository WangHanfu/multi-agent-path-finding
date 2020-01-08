classdef Robot<handle
    
    properties
        id
        type %[0=transport robot, 1=pick robot]
        State %[x,y,theta]
        
        Action %actions: 0=wait,1=forward,2=backward,3=turn left,4=turn right
        
        priority
        Path
        
        Mission
        Task
        Workzone %[xmin,xmax,ymin,ymax]
        status % 0=idle,1=on the road, 2=waiting, 3=mutual pick
        StatusRecord        
    end
    
    methods
        function obj = Robot()            
        end
        
        %Initialize the robot.
        function setAttribute(obj,id,type,initialState)
            obj.id = id;    
            obj.type = type;
            obj.setState(initialState);
            obj.priority = id; %default priority
        end
        
        %Set state.
        function setState(obj, state)
            obj.State = state;
        end
        
        %Get state.
        function state = getState(obj)
             state = obj.State;
        end
        
        %Set task.
        function setTask(obj, task)
            obj.Task = task;
        end
        
        %path planning
        function pathPlanning(obj)
            obj.Path;
        end
             
    end
end
