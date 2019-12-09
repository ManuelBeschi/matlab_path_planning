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
                        conn=Connection(nc,np);
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
    end
    
    
end