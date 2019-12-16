classdef Tree < handle
    
    properties
        root
        nodes
        direction
        max_distance
        tol
        collision_checker
        
        tree_plot_handles
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
        
        
        function cost=costToNode(obj,n)
            if ~obj.isInTree(n)
                warning('is not in tree');
                cost=inf;
                return
            end
            cost=0;
            if obj.direction
                while (~isempty(n.parent_connections))
                    if length(n.parent_connections)>1
                        error('a node of forward-direction tree should have only a parent');
                    end
                    cost=cost+n.parent_connections(1).getCost;
                    n=n.parent_connections(1).getParent;
                end
                if ~isequal(n,obj.root)
                    error('the node is not a member of the tree');
                end
            else
                while (~isempty(n.child_connections))
                    if length(n.child_connections)>1
                        error('a node of forward-direction tree should have only a parent');
                    end
                    cost=cost+n.parent_connections(1).getCost;
                    n=n.child_connections(1).getChild;
                end
                if ~isequal(n,obj.root)
                    error('the node is not a member of the tree');
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
        
        function res=keepOnlyThisBranch(obj,branch_connections)
            if isempty(branch_connections)
                res=false;
            end
            res=true;
            branch_nodes=branch_connections(1).getParent;
            for ib=1:length(branch_connections);
                branch_nodes=[branch_nodes;branch_connections(ib).getChild];
            end
            
            for in=1:length(obj.nodes)
                keep=false;
                for ib=1:length(branch_nodes)
                    if isequal(branch_nodes(ib),obj.nodes(in))
                        keep=true;
                        break;
                    end
                end
                if keep
                    continue;
                end
                delete(obj.nodes(in));
            end
            
        end
        
        function res=addBranch(obj,branch_connections)
            
            start_node=branch_connections(1).getParent;
            res=obj.isInTree(start_node);
            if ~res
                return
            end
            
            for ic=1:length(branch_connections)
                obj.nodes=[obj.nodes; branch_connections(ic).getChild];
            end
        end
        
        function res=incorporateLeafs(obj)
            res=obj.incorporateLeafFromNode(obj.root);
        end
        function res=incorporateLeafFromNode(obj,node)
            
%             res=obj.isInTree(node);
            res=true;
            if ~res
                return
            end
            
            if obj.direction
                for ic=1:length(node.child_connections)
                    c=node.child_connections(ic).getChild;
                    if ~obj.isInTree(c)
                        obj.nodes=[obj.nodes; c];
                    end
                    obj.incorporateLeafFromNode(c);
                end
            else
                for ip=1:length(node.parent_connections)
                    p=node.parent_connections(ip).getParent;
                    if ~obj.isInTree(p)
                        obj.nodes=[obj.nodes; p];
                    end
                    
                    obj.incorporateLeafFromNode(p);
                end
            end
        end
        
        function improved=rewire(obj,q,checker,r_rewire)
            if (~obj.direction)
                error('rewiring is available only on forward tree');
            end
            [success,new_node]=extend(obj,q);
            if ~success
                improved=false;
                return
            end
            improved=false;
            if nargin<4
                r_rewire=1;
            end
            near_nodes=obj.near(new_node,r_rewire);
            
            nearest_node=new_node.parent_connections(1).getParent;
            cost_to_new=obj.costToNode(new_node);
            for in=1:length(near_nodes)
                if isequal(near_nodes(in),nearest_node)
                    continue;
                end
                cost_to_near=obj.costToNode(near_nodes(in));
                if (cost_to_near>cost_to_new)
                    continue;
                end
                cost_near_to_new=norm(new_node.q-near_nodes(in).q);
                if ((cost_to_near+cost_near_to_new)>cost_to_new)
                    continue;
                end
                if (~checker.checkPath([near_nodes(in).q,new_node.q]))
                    continue;
                end
                new_node.parent_connections(1).delete;
                conn=Connection(near_nodes(in),new_node);
                nearest_node=near_nodes(in);
                cost_to_new=cost_to_near+cost_near_to_new;
                improved=true;
            end
            
            for in=1:length(near_nodes)
                cost_to_near=obj.costToNode(near_nodes(in));
                if cost_to_new>=cost_to_near
                    continue;
                end
                cost_new_to_near=norm(near_nodes(in).q-new_node.q);
                if (cost_to_new+cost_new_to_near)>cost_to_near
                    continue;
                end
                if (~checker.checkPath([new_node.q,near_nodes(in).q]))
                    continue;
                end
                
                near_nodes(in).parent_connections(1).delete;
                conn=Connection(new_node,near_nodes(in));
                improved=true;
            end
        end
        
        function nodes=near(obj,node,r_rewire)
            nodes=[];
            for in=1:length(obj.nodes)
                dist=obj.computeDistance(obj.nodes(in).q,node.q);
                if (dist<r_rewire)
                    nodes=[nodes,obj.nodes(in)];
                end
            end
        end
        
        function [res,in]=isInTree(obj,node)
            res=false;
            for in=1:length(obj.nodes)
                if isequal(node,obj.nodes(in))
                    res=true;
                    return;
                end
            end
        end
        
        function plot(obj)
            for ip=1:length(obj.tree_plot_handles)
                delete(obj.tree_plot_handles(ip))
            end
            for in=1:length(obj.nodes)
                if isvalid(obj.nodes(in))
                    obj.tree_plot_handles=[obj.tree_plot_handles obj.nodes(in).plot];
                end
            end
        end
        
        function distance=computeDistance(obj,q1,q2)
            distance=norm(q1-q2);
        end
    end
end

