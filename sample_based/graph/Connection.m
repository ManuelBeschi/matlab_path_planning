classdef Connection < handle
    %CONNECTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access=protected)
        parent
        child
        cost
    end
    
    methods
        function register(obj)
            obj.parent.addChildConnection(obj);
            obj.child.addParentConnection(obj);
        end

        function obj = Connection(parent,child,cost)
            obj.parent= parent;
            obj.child=child;
            assert(parent~=child);
            obj.register;
            obj.cost=cost;
            
        end
        
        function parent=getParent(obj)
            parent=obj.parent;
        end
        
        function child=getChild(obj)
            child=obj.child;
        end
        
        function setCost(obj,cost)
            obj.cost=cost;
        end
        function cost=getCost(obj)
            cost=obj.cost;
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

