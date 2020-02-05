classdef Path < handle
    % Path (series of connections)
    
    
    properties
        connections
        change_warp
        change_slipChild
        change_slipParent
        change_spiral
        debug_flag
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
            obj.change_warp=ones(length(connections),1);
            obj.change_slipChild=ones(length(connections),1);
            obj.change_slipParent=ones(length(connections),1);
%             obj.change_spiral=ones(length(connections),1);
            if ~isempty(connections)
                obj.change_warp(1)=0;
                obj.change_slipChild(1)=0;
                obj.change_slipParent(1)=0;
%                 obj.change_spiral(1)=0;
            end
            
            obj.debug_flag=0;
        end
        
        function cost = cost(obj)
            cost=0;
            if length(obj.connections)<1
                return
            end
            for ic=1:length(obj.connections)
                cost=cost+obj.connections(ic).getCost;
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
                return
            end
            for idx=1:length(obj.connections)
                if norm(node.q-obj.connections(idx).getChild.q)<1e-6
                    subpath=Path(obj.connections(idx+1:end));
                    return;
                    
                end
            end
            error('the node is not part of the path');
        end
        
        function new_path=resample(obj,distance,metrics)
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
                        conn=Connection(np,nc,metrics.cost(np,nc));
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
        
        function verboseDebug(obj,flag)
            obj.debug_flag=flag;
        end
        
        function improved=bisection(obj,connection_idx,center,direction,max_distance,min_distance,min_length,checker,metrics)
            parent=obj.connections(connection_idx-1).getParent;
            node=obj.connections(connection_idx).getParent;
            child=obj.connections(connection_idx).getChild;
            improved=false;
            distance=0.5*(min_distance+max_distance);
            cost=obj.connections(connection_idx-1).getCost+obj.connections(connection_idx).getCost;
            
            if obj.debug_flag
                ll=[];
            end
            iter=0;
            while (iter<100 && (max_distance-min_distance)>min_length)
                iter=iter+1;
                p=center+direction*distance;
                
                if obj.debug_flag
                    hh(1)=plot3(p(1),p(2),p(3),'om');
                    hh(2)=plot3(center(1),center(2),center(3),'*m');
                    hh(3)=plot3(parent.q(1),parent.q(2),parent.q(3),'dm');
                    hh(4)=plot3([parent.q(1) p(1) child.q(1)],[parent.q(2) p(2) child.q(2)],[parent.q(3) p(3) child.q(3)],'m');
                    pause(0.1)
                    delete(hh)
                end
                cost_pn=metrics.cost(parent.q,p);
                cost_nc=metrics.cost(p,child.q);
                costn=cost_pn+cost_nc;
                
                if costn>=cost
                    min_distance=distance;
                else
                    is_valid=checker.checkPath([parent.q p]) && checker.checkPath([p child.q]);
                    if (~is_valid)
                        min_distance=distance;
                    else
                        improved=true;
                        max_distance=distance;
                        cost=costn;
                        obj.connections(connection_idx).delete,
                        node=Node(p);
                        obj.connections(connection_idx-1)=Connection(parent,node,cost_pn);
                        obj.connections(connection_idx)=Connection(node,child,cost_nc);
                        
                        if obj.debug_flag
                            joints=obj.getWaypoints;
                            if ~isempty(ll)
                                delete(ll);
                            end
                            ll=plot3(joints(1,:)',joints(2,:)',joints(3,:)','LineWidth',2,'Color',[1 1 1]*0.8);
                        end
                    end
                end
                distance=0.5*(max_distance+min_distance);
            end
            if improved
                obj.change_warp(connection_idx+(-1:0))=1;
                obj.change_slipChild(connection_idx+(-1:0))=1;
%                 obj.change_spiral(connection_idx+(-1:0))=1;
                obj.change_slipParent(connection_idx+(-1:0))=1;
                obj.change_slipChild(1)=0;
                obj.change_slipParent(1)=0;
%                 obj.change_spiral(1)=0;
            end
        end
        
        function stall=slipChild(obj,checker,metrics)
            
            
            idxs=2:length(obj.connections);
            for ip=idxs
                
                if any(obj.change_slipChild(ip+(-1:0)))
                    
                    parent=obj.connections(ip-1).getParent;
                    node=obj.connections(ip).getParent;
                    child=obj.connections(ip).getChild;
                    
                    c=child.q;
                    dir=node.q-c;
                    max_distance=norm(dir);
                    
                    min_length=0.01;
                    if (norm(max_distance)<min_length)
                        obj.change_slipChild(ip)=0;
                        continue;
                    end
                    
                    v=dir/max_distance;
                    min_distance=0;
                    if (~obj.bisection(ip,c,v,max_distance,min_distance,min_length,checker,metrics))
                        obj.change_slipChild(ip)=0;
                    end
                end
            end
            
            stall=~any(obj.change_slipChild);
        end
        
        function stall=slipParent(obj,checker,metrics)
            
            idxs=2:length(obj.connections);
            for ip=idxs
                
                if any(obj.change_slipParent(ip+(-1:0)))
                    parent=obj.connections(ip-1).getParent;
                    node=obj.connections(ip).getParent;
                    child=obj.connections(ip).getChild;
                    
                    min_length=0.01;
                    
                    c=parent.q;
                    dir=node.q-c;
                    max_distance=norm(dir);
                    if (norm(max_distance)<min_length)
                        obj.change_slipParent(ip)=0;
                        continue;
                    end
                    
                    v=dir/max_distance;
                    min_distance=0;
                    if (~obj.bisection(ip,c,v,max_distance,min_distance,min_length,checker,metrics))
                        obj.change_slipParent(ip)=0;
                    end
                end
            end
            stall=~any(obj.change_slipParent);
            
        end
        
        function stall=spiral(obj,checker,metrics)
            idxs=2:length(obj.connections);
            for ip=idxs
                if any(obj.change_spiral(ip+(-1:0)))
                    parent=obj.connections(ip-1).getParent;
                    node=obj.connections(ip).getParent;
                    child=obj.connections(ip).getChild;
                    
                    min_length=0.01;
                    
                    c=(parent.q+child.q)*0.5;
                    dir=node.q-c;
                    max_distance=norm(dir);
                    if (norm(max_distance)<min_length)
                        obj.change_spiral(ip)=0;
                        continue;
                    end
                    
                    v1=dir/max_distance;
                    v2=-(child.q-parent.q)/norm(child.q-parent.q);
                    v2=v2-(v2'*v1)*v1;
                    v2=v2/norm(v2);
                    v3=randn(length(v1),1);
                    v3=v3-(v3'*v1)*v1;
                    v3=v3-(v3'*v2)*v2;
                    v3=v3/norm(v3);
                    %v=0.5*v1+0.5*v3;
                    v=0.1*v1+0.9*v3;
                    v=v/norm(v);
                    min_distance=0;
                    if (~obj.bisection(ip,c,v,max_distance,min_distance,min_length,checker,metrics))
                        obj.change_spiral(ip)=0;
                    end
                end
                
            end
            stall=~any(obj.change_spiral);
        end
        
        function stall=warp(obj,checker,metrics)
            
            idxs=2:length(obj.connections);
            for ip=idxs
                
                if any(obj.change_warp(ip+(-1:0)))
                    
                    parent=obj.connections(ip-1).getParent;
                    node=obj.connections(ip).getParent;
                    child=obj.connections(ip).getChild;
                    
                    c=(parent.q+child.q)*0.5;
                    dir=node.q-c;
                    max_distance=norm(dir);
                    min_length=0.01;
                    if (norm(max_distance)<min_length)
                        obj.change_warp(ip)=0;
                        continue;
                    end
                    
                    v=dir/max_distance;
                    min_distance=0;
                    
                    if (~obj.bisection(ip,c,v,max_distance,min_distance,min_length,checker,metrics))
                        obj.change_warp(ip)=0;
                    end
                end
            end
            stall=~any(obj.change_warp);
        end
        
        function simplified=simplify(obj,checker,metrics)
            ic=1;
            simplified=false;
            while ic<=length(obj.connections)
                dist=norm(obj.connections(ic).getParent.q-obj.connections(ic).getChild.q);
                min_length=0.01;
                if dist>min_length
                    ic=ic+1;
                    continue;
                end
                if ic==1
                    ic=ic+1;
                    continue;
                end
                if checker.checkPath([obj.connections(ic-1).getParent.q,obj.connections(ic).getChild.q])
                    simplified=true;
                    conn_cost=metrics.cost(obj.connections(ic-1).getParent.q,obj.connections(ic).getChild.q);
                    new_conn=Connection(obj.connections(ic-1).getParent, obj.connections(ic).getChild,conn_cost);
                    obj.connections(ic).delete
                    obj.connections=[obj.connections(1:(ic-2)) new_conn obj.connections((ic+1):end)];
                    
                    obj.change_warp=[obj.change_warp(1:(ic-1)); obj.change_warp((ic+1):end)];
                    obj.change_warp(ic-1)=1;
                    obj.change_slipParent=[obj.change_slipParent(1:(ic-1)); obj.change_slipParent((ic+1):end)];
                    obj.change_slipParent(ic-1)=1;
                    obj.change_slipChild=[obj.change_slipChild(1:(ic-1)); obj.change_slipChild((ic+1):end)];
                    obj.change_slipChild(ic-1)=1;
%                     
%                     obj.change_spiral=[obj.change_spiral(1:(ic-1)) obj.change_spiral((ic+1):end)];
%                     obj.change_spiral(ic-1)=1;
                    
                    
                else
                    ic=ic+1;
                end
            end
            
%             if simplified
%                 obj.change_warp=ones(length(obj.connections),1);
%                 obj.change_slipChild=ones(length(obj.connections),1);
%                 obj.change_slipParent=ones(length(obj.connections),1);
% %                 obj.change_spiral=ones(length(obj.connections),1);
%                 if ~isempty(obj.connections)
%                     obj.change_warp(1)=0;
%                     obj.change_slipChild(1)=0;
%                     obj.change_slipParent(1)=0;
% %                     obj.change_spiral(1)=0;
%                 end
%             end
        end
    end
    
    
end