classdef path < handle
    
    properties
        connections
    end
    
    methods
        function obj = path(connections)
            obj.connections= connections;
        end
        
        function cost = cost(obj)
            cost=0;
            if length(obj.connections)<1
                return
            end
            q0=obj.connections(1).getParent.q;
            for idx=1:length(obj.connections)
                q1=obj.connections(idx).getChild.q;
                cost=cost+norm(q1-q0);
                q0=q1;
            end
        end
        
        function waypoints=getWaypoints(obj)
            waypoints=[];
            if length(obj.connections)<1
                return
            end
            waypoints(:,end+1)=obj.connections(1).getParent.q;
            for idx=1:length(obj.connections)
                waypoints(:,end+1)=obj.connections(idx).getChild.q;
            end
        end

        
    end
end

