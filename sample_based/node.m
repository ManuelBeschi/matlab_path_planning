classdef node < handle
    %NODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        parent_connections
        child_connections
        q
        njnt
    end
    
    methods
        function obj = node(q)
            obj.q= q;
            obj.njnt=length(q);
        end
        
        
        
        function success=addParentConnection(obj,connection)
            if (isequal(connection.getChild,obj))
                obj.parent_connections=[obj.parent_connections;connection];
                success=true;
            else
                warning('try to add a connection with a different child');
                success=false;
            end
        end
        function success=addChildConnection(obj,connection)
            if (isequal(connection.getParent,obj))
                obj.child_connections=[obj.child_connections;connection];
                success=true;
            else
                warning('try to add a connection with a different parent');
                success=false;
            end
        end
        
        
        function success=removeParentConnection(obj,connection)
            for ic=1:length(obj.parent_connections)
                if (isequal(connection,obj.parent_connections(ic)))
                    obj.parent_connections(ic)=[];
                    success=true;
                    return;
                end
            end
            success=false;
        end
        
        
        function success=removeChildConnection(obj,connection)
            for ic=1:length(obj.child_connections)
                if (isequal(connection,obj.child_connections(ic)))
                    obj.child_connections(ic)=[];
                    success=true;
                    return;
                end
            end
            success=false;
        end
        
    end
end

