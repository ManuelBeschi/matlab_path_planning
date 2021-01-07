function [new_path,path_cost,success,connected2path_number,subpath_from_path2] = PathSwitch(current_path,other_paths,node,lb,ub,max_distance,checker,metrics,opt_type,succ_node)
% [new_path,path_cost,success,connected2path_number,subpath_from_path2] = PathSwitch(current_path,other_paths,node,lb,ub,max_distance,checker,metrics,opt_type,succ_node)
% OUTPUT:
%> new_path: the calculated path that starts from the current_path node and moves to another path of other_paths
%> path_cost: the cost of the new planned path
%> success: 1 if the path switch is possible, 0 otherwise.
%> connected2path_number: the number of the path (path2) of the other_paths set to which the new planned path is connected
%> subpath_from_path2: the subpath of path2 starting from the node to which the new planned path is connected
% INPUT:
%> current_path: the path currently traveled
%> other_paths: vector of the other available paths
%> node: node from which the switch starts
%> lb, ub: lower and upper joints bounds
%> max_distance: the max distance between node admissible for the BirrtExtend algorithm
%> checker: the collision checker
%> metrics: the metric used to calculate the connections cost
%> opt_type: the type of path local optimization
%> succ_node: if 1, the PathSwitch function looks for the best node of
% all the other paths to connect to, oherwise it considers, for any other
% path, the closest node.

verbose = 0;

new_path = [];
success = 0;
connected2path_number = 0;
subpath_from_path2 = [];

%Identifying the subpath of current_path starting from node
path1_node = node;
path1_node2goal = current_path.getSubpathFromNode(path1_node);
subpath1_cost = path1_node2goal.cost;

path_cost = subpath1_cost;

for j = 1:length(other_paths)
    
    path2 = other_paths(j);
    
    %Finding the closest node
    path2_node_vector = path2.findCloserNode(path1_node.q);
    if(eq(path2_node_vector.q,path2.connections(end).getChild.q)) %if the cloest node is the GOAL, the second closest node is considered
        path_support = path2.getSubpathToNode(path2.connections(end).getParent);
        path2_node_vector = path_support.findCloserNode(path1_node.q);
    end
    
    if(succ_node == 1)
        path2_conn = path2.connections;
        path2_node_vector = [path2_node_vector,path2_conn(1).getParent]; %#ok<AGROW> The first node is the closest one, then there are the others. In this way you are trying to exclude the longest paths because you search a path first to the closest node than to the others
        for t=2:length(path2_conn)  %the GOAL is not considered as a node to which directly connecting the path
            if(norm(path2_conn(t).getParent.q-path2_node_vector(end).q)>1e-03 && eq(path2_conn(t).getParent,path2_node_vector(1)) == 0) %when some nodes are too close to each other, only one of them is considered; the closest node is excluded because already present in the first element of the array 
                path2_node_vector = [path2_node_vector,path2_conn(t).getParent]; %#ok<AGROW> 
            end
        end
    end
    
%     if(succ_node == 1)     %CON SUCCNODE == 1 NON METTE IL NODO PIU VICINO PRIMO
%         path2_conn = path2.connections;
%         path2_node_vector = [path2_conn(1).getParent];
%         for t=2:length(path2_conn)  %the GOAL is not considered as a node to which directly connecting the path
%             if(norm(path2_conn(t).getParent.q-path2_node_vector(end).q)>1e-03) %when some nodes are too close to each other, only one of them is considered
%                 path2_node_vector = [path2_node_vector,path2_conn(t).getParent]; %#ok<AGROW> 
%             end
%         end
%     else
%         path2_node_vector = path2.findCloserNode(path1_node.q);
%         if(eq(path2_node_vector.q,path2.connections(end).getChild.q)) %if the cloest node is the GOAL, the second closest node is considered
%             path_support = path2.getSubpathToNode(path2.connections(end).getParent);
%             path2_node_vector = path_support.findCloserNode(path1_node.q);
%         end
%     end
    
    for k=1:length(path2_node_vector)
 
            path2_node = path2_node_vector(k);

            path2_node2goal = path2.getSubpathFromNode(path2_node);
            subpath2 = path2_node2goal.connections;
            subpath2_cost = path2_node2goal.cost;
            
            diff_subpath_cost = path_cost-subpath2_cost;  %it is the maximum cost for connecting_path to be convenient
            distance_path_node = norm(path1_node.q-path2_node.q,2); %the Euclidean distance is the minimum cost that the connecting_path can have
            
            if (distance_path_node<diff_subpath_cost) %if the Euclidean distance between the two nodes is bigger than the maximum cost for the connecting_path to be convenient, it isuseless to calculate a connecting_path because it surely will not be convenient
                
                path1_node_fake = Node(path1_node.q);
                path2_node_fake = Node(path2_node.q);
            
                replan_sampler = InformedSampler(path1_node_fake.q,path2_node_fake.q,lb,ub,diff_subpath_cost); %the ellipsoide determined by diff_subpath_cost is used. Outside from this elipsoid, the nodes create a connecting_path not convenient
                %replan_sampler.setCost(diff_subpath_cost)

                solver_replannig = BirrtExtend(path1_node_fake,path2_node_fake,max_distance,checker,replan_sampler,metrics);
                [solved,replanned_path] = solver_replannig.solve;
                
                if(solved)
                    replanned_path.verboseDebug(false);
                    path_optimizer=PathLocalOptimizer(replanned_path,opt_type,checker,metrics);
                    path_optimizer.solve; 
                    connecting_path = path_optimizer.path.connections;
    
                    conn_cost = subpath2_cost;
                    for i=1:length(connecting_path) %calculating the cost of the solution found
                        conn_cost=conn_cost+metrics.cost(connecting_path(i).getParent,connecting_path(i).getChild);
                    end
                    
                    if(verbose)
                        disp('path number, node number:') %#ok<*UNRCH>
                        disp(j);
                        disp(k);
                        disp('cost:')
                        disp(conn_cost)
                    end
                    
                    if(conn_cost<path_cost && conn_cost<subpath1_cost && ~isempty(subpath2)) %maybe "if(conn_cost<path_cost)" is enough (the GOAL is always excluded and at the beginning subpath1_cost is path_cost)
                            
                        if (length(connecting_path)>1)
                            node1 = connecting_path(1).getChild;
                            node2 = connecting_path(end).getParent;
                            conn1_cost = metrics.cost(path1_node,node1);
                            conn2_cost = metrics.cost(node2,path2_node);
                            conn1 = Connection(path1_node,node1,conn1_cost);
                            conn2 = Connection(node2,path2_node,conn2_cost);
                            
                            connecting_path = [conn1,connecting_path(2:end-1),conn2];
                       
                        else 
                            conn1_cost = metrics.cost(path1_node,path2_node);
                            conn1 = Connection(path1_node,path2_node,conn1_cost);
                            connecting_path = conn1;
                        end
                        
                        path1_node_fake.delete;
                        path2_node_fake.delete;
                        
                        new_path=Path([connecting_path subpath2]);
                        path_cost = conn_cost;
                        success = 1;
                        if(nargout>3)
                            connected2path_number = j;
                        end
                        if(nargout>4)
                            subpath_from_path2 = path2_node2goal;
                        end
                        if(verbose)
                            switch2path = j;
                            switch2node = k;
                            disp('Connection  POSSIBLE')
                            disp('-------------------------------------------');
                        end
                    end 
                else
                    if(verbose)
                        disp('Connection NOT POSSIBLE to the node number:')
                        disp(k);
                        disp('of the path number:')
                        disp(j);
                        disp('-------------------------------------------');
                    end
                end
            else
                if(verbose)
                    disp('It is better subpath1 than subpath2')
                    disp('path number:')
                    disp(j);
                    disp('node number:')
                    disp(k);
                    disp(diff_subpath_cost)
                    disp(distance_path_node)
                    disp('-------------------------------------------');
                end
            end
    end

end

if(verbose)
    disp('Connection to the path number:')
    disp(switch2path)
    disp('-------------------------------------------');
    if(succ_node == 1)
        disp('To the node number:')
        disp(switch2node)
        disp('-------------------------------------------');
    end
    disp('cost:')
    disp(path_cost)
end
end

