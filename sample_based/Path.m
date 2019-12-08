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
                    return;p
                    
                    
                    
                    
               end
            end
            error('the node is not part of the path');
        end

    end
    
end