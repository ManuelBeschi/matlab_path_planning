classdef Node < handle
    %NODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        parent_connections
        child_connections
        q
        njnt
    end
    
    methods
        function obj = Node(q)
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
            if (~isvalid(obj))
                success=true;
                return;
            end
            for ic=1:length(obj.parent_connections)
                if (isequal(connection,obj.parent_connections(ic)))
                    obj.parent_connections(ic)=[];
                    success=true;
                    return;
                end
            end
            success=false;
        end
        
        function delete(obj)
            for ic=1:length(obj.parent_connections)
                if isvalid(obj.parent_connections(ic))
                    delete(obj.parent_connections(ic));
                end
            end
            for ic=1:length(obj.child_connections)
                if isvalid(obj.child_connections(ic))
                    delete(obj.child_connections(ic));
                end
            end
        end
        
        function success=removeChildConnection(obj,connection)
            if (~isvalid(obj))
                success=true;
                return;
            end
            for ic=1:length(obj.child_connections)
                if (isequal(connection,obj.child_connections(ic)))
                    obj.child_connections(ic)=[];
                    success=true;
                    return;
                end
            end
            success=false;
        end
        
        function line_handle=plot(obj)
            line_handle=[];
            if (length(obj.q)==3)
                line_handle=[line_handle plot3(obj.q(1),obj.q(2),obj.q(3),'o','Color',[1 1 1]*0.5)];
                for ic=1:length(obj.child_connections)
                    c=obj.child_connections(ic).getChild.q;
                    v=c-obj.q;
                    line_handle=[line_handle plot3(obj.q(1)+[0 v(1)],obj.q(2)+[0 v(2)],obj.q(3)+[0 v(3)],'Color',[1 1 1]*0.5)];
                end
            end
            
        end
        
    end
end

