classdef Path < handle
    % Path (series of connections)
    
    
    properties
        connections
    end
    
    methods
        function obj = Path(connections)
            if isempty(connections)
                warning('empty path')
            end
            if isa(connections,'Connection')
                obj.connections= connections;
            else
                error('connections is not a vector of Connections');
            end
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
        
        function [closest_node]=findCloserNode(obj,q)
            if length(obj.connections)<1
                closest_node=[];
                return
            end
            if isa(q,'Node')
                q=q.q;
            end
            wp=obj.getWaypoints;
            min_dist=inf;
            i_closer=1;
            for idx=1:length(wp)
                dist=norm(wp(:,idx)-q);
                if (dist<min_dist)
                    min_dist=dist;
                    i_closer=idx;
                end
            end
            if i_closer==1
                closest_node=obj.connections(1).getParent;
                return;
            end
            closest_node=obj.connections(i_closer-1).getChild;
            
        end
        
        function subpath=getSubpathToNode(obj,node)
            if norm(node.q-obj.connections(1).getParent.q)<1e-6
                warning('path is empty');
                subpath=Path([]);
            end
            for idx=1:length(obj.connections)
                if norm(node.q-obj.connections(idx).getChild.q)<1e-6
                    subpath=Path(obj.connections(1:idx));
                    return;
                end
            end
            error('the node is not part of the path');
        end
        
        function subpath=getSubpathFromNode(obj,node)
            if norm(node.q-obj.connections(1).getParent.q)<1e-6
                subpath=obj;
            end
            for idx=1:length(obj.connections)
                if norm(node.q-obj.connections(idx).getChild.q)<1e-6
                    subpath=Path(obj.connections(idx+1:end));
                    return;
                    
                end
            end
            error('the node is not part of the path');
        end
        
        function new_path=resample(obj,distance)
            new_connections=[];
            for idx=1:length(obj.connections)
                n2=obj.connections(:,idx).getParent;
                n1=obj.connections(:,idx).getChild;
                dist=norm(n1.q-n2.q);
                npnt=ceil(dist/distance);
                if (npnt==1)
                    new_connections=[new_connections obj.connections(idx)];
                else
                    np=n2;
                    for ip=1:npnt
                        if (ip==npnt)
                            nc=n1;
                        else
                            pos=n2.q+(n1.q-n2.q)*(ip)/npnt;
                            nc=Node(pos);
                        end
                        conn=Connection(np,nc);
                        new_connections=[new_connections conn];
                        np=nc;
                    end
                end
            end
            new_path=Path(new_connections);
        end
        
        function plot(obj)
            joints=obj.getWaypoints;
            s=zeros(size(joints,2),1);
            for idx=2:length(s)
                s(idx)=s(idx-1)+norm(joints(:,idx)-joints(:,idx-1));
            end
            plot(s,joints')
            xlabel('curve length');
            ylabel('joint configuration');
        end
        
        function cost_evolution=localOptimization(obj,checker)
            
            alpha=1;
            gamma=0.1;
            cost_evolution=[];
            for igen=1:10
                for ip=2:length(obj.connections)
                    
                    parent=obj.connections(ip-1).getParent;
                    node=obj.connections(ip).getParent;
                    child=obj.connections(ip).getChild;
                    stall=0;
                    
                    dir=randn(length(node.q),1);
                    dir=dir/norm(dir);
                    
                    for itrial=1:10
                        if (stall>5)
                            break;
                        end
                        stall=stall+1;
                        cost=norm(node.q-parent.q)+norm(child.q-node.q); % usa costo generico
                        
                        
                        q=node.q;
                        qn=q+gamma*dir;
                        cost_n=norm(qn-parent.q)+norm(child.q-qn); % usa costo generico
                        dc=(cost_n-cost)/gamma;
                        if abs(dc)<1e-6
                            dir=randn(length(node.q),1);
                            dir=dir/norm(dir);
                            continue;
                        end
                        qp=q-alpha*dc*dir;
                        cost_n=norm(qp-parent.q)+norm(child.q-qp); % usa costo generico
                        if (cost_n>cost)
                            dir=randn(length(node.q),1);
                            dir=dir/norm(dir);
                            continue;
                        end
                        is_valid=checker.checkPath([parent.q qp]) && checker.checkPath([qp child.q]);
                        if ~is_valid
                            dir=randn(length(node.q),1);
                            dir=dir/norm(dir);
                            continue;
                        end
                        obj.connections(ip-1).delete;
                        obj.connections(ip).delete,
                        node.delete;
                        node=Node(qp);
                        obj.connections(ip-1)=Connection(parent,node);
                        obj.connections(ip)=Connection(node,child);
                        cost_evolution(end+1,1)=obj.cost;
                        stall=0;
                    end
                end
            end
        end
        
        function cost_evolution=localOptimization2(obj,checker)
            
            alpha=1;
            gamma=0.1;
            cost_evolution=[];
            for igen=1:100
                dir=randn(length(obj.connections(1).getParent.q),1);
                dir=dir/norm(dir);
                
                for ip=2:length(obj.connections)
                    
                    parent=obj.connections(ip-1).getParent;
                    node=obj.connections(ip).getParent;
                    child=obj.connections(ip).getChild;
                    stall=0;
                    
                    
                    for itrial=1:50
                        if (stall>5)
                            break;
                        end
                        stall=stall+1;
                        cost=norm(node.q-parent.q)+norm(child.q-node.q); % usa costo generico
                        
                        
                        q=node.q;
                        qn=q+gamma*dir;
                        cost_n=norm(qn-parent.q)+norm(child.q-qn); % usa costo generico
                        dc=(cost_n-cost)/gamma;
                        if abs(dc)<1e-6
                            continue;
                        end
                        %qp=q-alpha*dc*dir;
                        qp=q-alpha*sign(dc)*dir;
                        cost_n=norm(qp-parent.q)+norm(child.q-qp); % usa costo generico
                        if (cost_n>0.99*cost)
                            continue;
                        end
                        is_valid=checker.checkPath([parent.q qp]) && checker.checkPath([qp child.q]);
                        if ~is_valid
                            continue;
                        end
                        obj.connections(ip-1).delete;
                        obj.connections(ip).delete,
                        node.delete;
                        node=Node(qp);
                        obj.connections(ip-1)=Connection(parent,node);
                        obj.connections(ip)=Connection(node,child);
                        cost_evolution(end+1,1)=obj.cost;
                        stall=0;
                    end
                end
            end
        end
        
        
        function cost_evolution=slipChild(obj,checker)
            
             cost_evolution=[];
            for itrial=1:1
                idxs=2:length(obj.connections);
                for ip=idxs
                    
                    parent=obj.connections(ip-1).getParent;
                    node=obj.connections(ip).getParent;
                    child=obj.connections(ip).getChild;
                    
                    c=child.q;
                    dir=node.q-c;
                    max_distance=norm(dir);
                    
                    min_length=0.01;
                    if (norm(max_distance)<min_length)
                        continue;
                    end
                    
                    v=dir/max_distance;
                    
%                     p=node.q;
%                     quiver3(p(1),p(2),p(3),v(1),v(2),v(3));
                    
                    iter=0;
                    min_distance=0;
                    distance=0.5*max_distance;
                    cost=norm(node.q-parent.q)+norm(child.q-node.q); % usa costo generico
                    while (iter<100 && (max_distance-min_distance)>min_length)
                        iter=iter+1;
                        p=c+v*distance;
                        costn=norm(parent.q-p)+norm(p-child.q);
                        if costn>cost
                            min_distance=distance;
                        else
                            is_valid=checker.checkPath([parent.q p]) && checker.checkPath([p child.q]);
                            if (~is_valid)
                                min_distance=distance;
                            else
                                max_distance=distance;
                                cost=costn;
                                obj.connections(ip-1).delete;
                                obj.connections(ip).delete,
                                node.delete;
                                node=Node(p);
                                obj.connections(ip-1)=Connection(parent,node);
                                obj.connections(ip)=Connection(node,child);
                                cost_evolution(end+1,1)=obj.cost;
                            end
                        end
                        distance=0.5*(max_distance+min_distance);
                    end
                end
                
            end
        end
        
        
        function cost_evolution=slipParent(obj,checker)
            
             cost_evolution=[];
            for itrial=1:1
                idxs=2:length(obj.connections);
                for ip=idxs
                    
                    parent=obj.connections(ip-1).getParent;
                    node=obj.connections(ip).getParent;
                    child=obj.connections(ip).getChild;
                    
                    min_length=0.01;
                    
                    c=parent.q;
                    dir=node.q-c;
                    max_distance=norm(dir);
                    if (norm(max_distance)<min_length)
                        continue;
                    end
                    
                    v=dir/max_distance;
                    
%                     p=node.q;
%                     quiver3(p(1),p(2),p(3),v(1),v(2),v(3));
                    
                    iter=0;
                    min_distance=0;
                    distance=0.5*max_distance;
                    cost=norm(node.q-parent.q)+norm(child.q-node.q); % usa costo generico
                    while (iter<100 && (max_distance-min_distance)>min_length)
                        iter=iter+1;
                        p=c+v*distance;
                        costn=norm(parent.q-p)+norm(p-child.q);
                        if costn>cost
                            min_distance=distance;
                        else
                            is_valid=checker.checkPath([parent.q p]) && checker.checkPath([p child.q]);
                            if (~is_valid)
                                min_distance=distance;
                            else
                                max_distance=distance;
                                cost=costn;
                                obj.connections(ip-1).delete;
                                obj.connections(ip).delete,
                                node.delete;
                                node=Node(p);
                                obj.connections(ip-1)=Connection(parent,node);
                                obj.connections(ip)=Connection(node,child);
                                cost_evolution(end+1,1)=obj.cost;
                            end
                        end
                        distance=0.5*(max_distance+min_distance);
                    end
                end
                
            end
        end
        
        function cost_evolution=spiral(obj,checker)
            
             cost_evolution=[];
            for itrial=1:1
                idxs=2:length(obj.connections);
                for ip=idxs
                    
                    parent=obj.connections(ip-1).getParent;
                    node=obj.connections(ip).getParent;
                    child=obj.connections(ip).getChild;
                    
                    min_length=0.01;
                    
                    c=parent.q;
                    dir=node.q-c;
                    max_distance=norm(dir);
                    if (norm(max_distance)<min_length)
                        continue;
                    end
                    
                    v1=dir/max_distance;
                    v2=-(child.q-parent.q)/norm(child.q-parent.q);
                    v3=cross(v1,v2);
                    v=0.5*v1+0.5*sign(randn)*v3;
                    v=v/norm(v);
                    
                    p=node.q;
                    %quiver3(p(1),p(2),p(3),v(1),v(2),v(3));
                    
                    iter=0;
                    min_distance=0;
                    distance=0.5*max_distance;
                    cost=norm(node.q-parent.q)+norm(child.q-node.q); % usa costo generico
                    while (iter<100 && (max_distance-min_distance)>min_length)
                        iter=iter+1;
                        p=c+v*distance;
                        costn=norm(parent.q-p)+norm(p-child.q);
                        if costn>cost
                            min_distance=distance;
                        else
                            is_valid=checker.checkPath([parent.q p]) && checker.checkPath([p child.q]);
                            if (~is_valid)
                                min_distance=distance;
                            else
                                max_distance=distance;
                                cost=costn;
                                obj.connections(ip-1).delete;
                                obj.connections(ip).delete,
                                node.delete;
                                node=Node(p);
                                obj.connections(ip-1)=Connection(parent,node);
                                obj.connections(ip)=Connection(node,child);
                                cost_evolution(end+1,1)=obj.cost;
                            end
                        end
                        distance=0.5*(max_distance+min_distance);
                    end
                end
                
            end
        end
        
        
        function cost_evolution=warp(obj,checker)
            
            cost_evolution=[];
            for itrial=1:1
                idxs=2:length(obj.connections);
                %idxs=idxs(randperm(length(idxs)));
                for ip=idxs
                    
                    parent=obj.connections(ip-1).getParent;
                    node=obj.connections(ip).getParent;
                    child=obj.connections(ip).getChild;
                    
                    c=(parent.q+child.q)*0.5;
                    dir=node.q-c;
                    max_distance=norm(dir);
                    min_length=0.01;
                    if (norm(max_distance)<min_length)
                        continue;
                    end
                    
                    v=dir/max_distance;

                    iter=0;
                    min_distance=0;
                    distance=0.5*max_distance;
                    cost=norm(node.q-parent.q)+norm(child.q-node.q); % usa costo generico
                    while (iter<100 && (max_distance-min_distance)>min_length)
                        iter=iter+1;
                        p=c+v*distance;
                        costn=norm(parent.q-p)+norm(p-child.q);
                        if costn>cost
                            min_distance=distance;
                        else
                            is_valid=checker.checkPath([parent.q p]) && checker.checkPath([p child.q]);
                            if (~is_valid)
                                min_distance=distance;
                            else
                                max_distance=distance;
                                cost=costn;
                                obj.connections(ip-1).delete;
                                obj.connections(ip).delete,
                                node.delete;
                                node=Node(p);
                                obj.connections(ip-1)=Connection(parent,node);
                                obj.connections(ip)=Connection(node,child);
                                cost_evolution(end+1,1)=obj.cost;
                            end
                        end
                        distance=0.5*(max_distance+min_distance);
                    end
                end
                
            end
        end
        
        
        function cost_evolution=localOptimization3(obj,checker)
            
            alpha=1;
            gamma=0.1;
            
            cost_evolution=[];
            cost_evolution(end+1,1)=obj.cost;
                        
            for igen=1:100
                
                for ip=2:length(obj.connections)
                    
                    parent=obj.connections(ip-1).getParent;
                    node=obj.connections(ip).getParent;
                    child=obj.connections(ip).getChild;
                    stall=0;
                    
                    dir=node.q-(parent.q+child.q)*0.5;
                    if norm(dir)<1e-6
                        dir=randn(length(dir),1);
                    end
                    dir=dir/norm(dir);
                    
                    for itrial=1:50
                        if (stall>5)
                            break;
                        end
                        stall=stall+1;
                        cost=norm(node.q-parent.q)+norm(child.q-node.q); % usa costo generico
                        
                        
                        q=node.q;
                        qn=q+gamma*dir;
                        cost_n=norm(qn-parent.q)+norm(child.q-qn); % usa costo generico
                        dc=(cost_n-cost)/gamma;
                        if abs(dc)<1e-6
                            break;
                        end
                        %qp=q-alpha*dc*dir;
                        qp=q-gamma*sign(dc)*dir;
                        cost_n=norm(qp-parent.q)+norm(child.q-qp); % usa costo generico
                        if (cost_n>0.999*cost)
                            break;
                        end
                        is_valid=checker.checkPath([parent.q qp]) && checker.checkPath([qp child.q]);
                        if ~is_valid
                            break;
                        end
                        obj.connections(ip-1).delete;
                        obj.connections(ip).delete,
                        node.delete;
                        node=Node(qp);
                        obj.connections(ip-1)=Connection(parent,node);
                        obj.connections(ip)=Connection(node,child);
                        cost_evolution(end+1,1)=obj.cost;
                        stall=0;
                    end
                end
            end
        end
        
    end
    
    
end