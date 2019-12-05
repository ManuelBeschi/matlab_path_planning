classdef connection
    %CONNECTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access=protected)
        parent
        child
    end
    
    methods
        function register(obj)
            obj.parent.addChildConnection(obj);
            obj.child.addParentConnection(obj);
        end

        function obj = connection(parent,child)
            obj.parent= parent;
            obj.child=child;
            obj.register;
        end
        
        function parent=getParent(obj)
            parent=obj.parent;
        end
        
        function child=getChild(obj)
            child=obj.child;
        end
        
        function delete(obj)
            if (~obj.parent.removeChildConnection(obj))
                warning('this connection is not registered well')
            end
            if (~obj.child.removeParentConnection(obj))
                warning('this connection is not registered well')
            end
            
        end
    end
end

