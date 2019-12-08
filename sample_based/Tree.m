classdef Tree < handle
    
    properties
        root
        nodes
        direction
        max_distance
        tol
        collision_checker
    end
    
    methods
        function obj = Tree(root,direction,max_distance,collision_checker)
            if (~isa(root,'Node'))
                error('root has to be a Node object')
            end
            obj.root=root;
            obj.nodes=[root];
            obj.direction=direction;
            obj.max_distance=max_distance;
            obj.collision_checker=collision_checker;
            obj.tol=1e-6;
        end
        
        function [success,qnext,closest_node] = tryExtend(obj,q)
            closest_node=findCloserNode(obj,q);
            if (isempty(closest_node))
                success=false;
                qnext=[];
                return;
            end
            
            if (norm(closest_node.q-q)<obj.tol)
                success=true;
                qnext=q;
                return
            elseif (norm(closest_node.q-q)<obj.max_distance)
                qnext=q;
            else
                qnext=closest_node.q+(q-closest_node.q)/norm(closest_node.q-q)*obj.max_distance;
            end
            
            if (~obj.collision_checker.checkPath([closest_node.q,qnext]))
                success=false;
                return
            end
            
            success=true;
        end
        
        function [success,new_node]=extend(obj,q)
            
            [success,qnext,closest_node]=obj.tryExtend(q);
            if (~success)
                new_node=[];
                return;
            end
            
            new_node=Node(qnext);
            if (obj.direction)
                new_conn=Connection(closest_node,new_node);
            else
                new_conn=Connection(new_node,closest_node);
            end
            obj.nodes=[obj.nodes;new_node];
            
        end
        
        function [success,new_node] = extendToNode(obj,n)
            
            [success,qnext,closest_node]=obj.tryExtend(n.q);
            if (~success)
                new_node=[];
                return;
            end
            
            success=true;
            if norm(n.q-qnext)<obj.tol
                new_node=n;
            else
                new_node=Node(qnext);
            end
            if (obj.direction)
                new_conn=Connection(closest_node,new_node);
            else
                new_conn=Connection(new_node,closest_node);
            end
            obj.nodes=[obj.nodes;new_node];
        end
        
        function [success,new_node] = connect(obj,q)
            success=true;
            new_node=[];
            while success
                [success,tmp_node]=obj.extend(q);
                if (success)
                    new_node=tmp_node;
                    if norm(new_node.q-q)<obj.tol
                        return;
                    end
                end
            end
            
        end
        
        function [success,new_node]=connectToNode(obj,n)
            success=true;
            new_node=[];
            while success
                [success,tmp_node]=obj.extendToNode(n);
                if (success)
                    new_node=tmp_node;
                     if norm(new_node.q-n.q)<obj.tol
                        return;
                    end
                end
            end
        end
        
        
        function closest_node=findCloserNode(obj,q)
            min_distance=inf;
            closest_node=[];
            for in=1:length(obj.nodes)
                dist=obj.computeDistance(obj.nodes(in).q,q);
                if (dist<min_distance)
                    min_distance=dist;
                    closest_node=obj.nodes(in);
                end
            end
        end
        
        function connections=getConnectionToNode(obj,n)
            connections=[];
            if (obj.direction)
                while (~isempty(n.parent_connections))
                    if length(n.parent_connections)>1
                        error('a node of forward-direction tree should have only a parent');
                    end
                    connections=[connections,n.parent_connections(1)];
                    n=n.parent_connections(1).getParent;
                end
                if ~isequal(n,obj.root)
                    error('the node is not a member of the tree');
                end
                connections=fliplr(connections);
            else
                while (~isempty(n.child_connections))
                    if length(n.child_connections)>1
                        error('a node of forward-direction tree should have only a parent');
                    end
                    connections=[connections,n.child_connections(1)];
                    n=n.child_connections(1).getChild;
                end
                if ~isequal(n,obj.root)
                    error('the node is not a member of the tree');
                end
            end
        end
        
        
        function distance=computeDistance(obj,q1,q2)
            distance=norm(q1-q2);
        end
    end
end

