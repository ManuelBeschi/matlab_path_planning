function [replanned_path,replanned_path_cost,success,replanned_path_vector,number_replanning, time_first_sol] = InformedOnlineReplanning(current_path,other_paths,q,lb,ub,max_distance,checker,metrics,opt_type,succ_node,informed,verbose)
% [replanned_path,replanned_path_cost,success,replanned_path_vector,number_replanning, time_first_sol] = InformedOnlineReplanning(current_path,other_paths,q,lb,ub,max_distance,checker,metrics,opt_type,succ_node,informed,verbose)
% OUTPUT:
%> replanned_path: the calculated path that minimizes the cost
%> replanned_path_cost: the cost of the new planned path
%> success: 1 if the algorithm succeeds, 0 otherwise
%> replanned_path_vector: the vector containing the 10 best paths found.
%> numbert_replanning: set of examined nodes (i.e nodes from which the replanning has been started)
%> time_first_sol: time required to find a first feasible solution
% INPUT:
%> current_path: the path currently traveled
%> other_paths: vector of the other available paths
%> q: actual robot configuration  
%> lb, ub: lower and upper joints bounds
%> max_distance: the max distance between node admissible for the BirrtExtend algorithm
%> checker: the collision checker
%> metrics: the metric used to calculate the connections cost
%> opt_type: the type of path local optimization
%> succ_node: if 1, the PathSwitch function looks for the best node of
% all the other paths to connect to, oherwise it considers, for any other
% path, the closest node.
%> informed: if 0, the algorithm is not informed (always current_path in
%PathSwitch). If 1, the algorithm is informed (replanned_path in PathSwitch)
%but the node_vector analyzed is not updated with the new nodes of the new
%path. If 2, the node vector is updated with the new nodes added.
%> verbose: if 0, no comments displayed, if 1 comment displayed, if 2
%comment and updated graph for each iteration displayed. Red square: actual
%node analyzed; cyan square: last starting node for replanning; cyan path:
%last replanned path

tic

replanned_path = [];
replanned_path_vector = [];
replanned_path_cost = inf;
previous_cost = current_path.cost;
success = 0;
flag_other_paths = 0;
examined_nodes = [];
cont = 0;
number_replanning = 0;

index = [];
idx = current_path.findConnection(q); %the connectiong of the robot current configuration
admissible_current_path = [];

first_sol = 1; %flag to calculate the first solution time

if(verbose > 0)
    previous_joints = [];
    old_node = [];
    nodesPlot_vector = [];
    node_number = 0;
    disp('idx:')
    disp(idx)
    
    if(verbose == 2)
        examined_nodes_plot = [];
        start_conf = current_path.connections(1).getParent.q;
        goal_conf = current_path.connections(end).getChild.q;     
    end
end

if(idx>0)
    path1_node_vector = [];
    parent = current_path.connections(idx).getParent;
    child = current_path.connections(idx).getChild;
    
    if(norm(parent.q-q,2)<1e-6) %if the current conf is too close to the parent or to the child, it is approximated with the parent/child
        actual_node = parent;
    elseif(norm(child.q-q,2)<1e-6)
        actual_node = child;
    else
        actual_node = Node(q);
    end
    
    if(current_path.cost == inf) %if the path is obstructed by an obstacle, the connection obstructed cost is infinte
        z = length(current_path.connections);  
        while(z>idx) %to find the savable part of current_path, the subpath after the connection obstruced by the obstacle
            if(current_path.connections(z).getCost == inf)
                if(z == length(current_path.connections))
                    admissible_current_path = []; %no savable subpath available
                else
                    admissible_current_path = current_path.getSubpathFromNode(current_path.connections(z).getChild);
                end
                z = 0;
            else
                z = z-1;
            end
        end
        
        if(isa(admissible_current_path,'Path'))
            other_paths = [admissible_current_path,other_paths];  %adding the savable subpath of the current_path to the set of available paths
        end
    end
    
    reset_other_paths = other_paths; %it will be useful later to reset the set of available paths to the initial set 
    
    if(current_path.connections(idx).getCost == inf || idx == length(current_path.connections)) %if the obstacle is obstructing the current connection or the current connection is the last one, the replanning must start from the current configuration, so a node corresponding to the config is added
        node = actual_node;
        cost_parent = metrics.cost(parent,node);
        conn_parent=Connection(parent,node,cost_parent);
        %cost_child = metrics.cost(node,child);
        cost_child = inf; 
        conn_child=Connection(node,child,cost_child);
        subpath_parent = current_path.getSubpathToNode(parent);
        subpath_child = current_path.getSubpathFromNode(child);
        node_conn = [conn_parent,conn_child];
        current_path = Path([subpath_parent.connections,node_conn,subpath_child.connections]);
        
        replanned_path = current_path.getSubpathFromNode(node);  %at the start, the replanned path is initialized with the subpath of the current path from the current config to the goal
        replanned_path_cost = replanned_path.cost;
        
        available_nodes = 0;
        limit = 1;
        path1_node_vector = node;
    else %if the current connection is free, all the nodes between the current child to the parent of the connection obstructed are considered as starting points for the replanning
        node = child; 
        actual_node_conn_cost = metrics.cost(actual_node,node);
        actual_node_conn = Connection(actual_node,node,actual_node_conn_cost);
        subpath1 = current_path.getSubpathFromNode(node);
        subpath1_conn = subpath1.connections;

        replanned_path = Path([actual_node_conn,subpath1_conn]); %at the start, the replanned path is initialized with the subpath of the current path from the current config to the goal
        replanned_path_cost = replanned_path.cost;
        
        for i=1:length(subpath1_conn)
            path1_node_vector = [path1_node_vector,subpath1_conn(i).getParent]; %#ok<AGROW>
            if(subpath1_conn(i).getCost == inf)
                index = [index,i]; %#ok<AGROW>
            end
        end
        if isempty(index)
            limit = length(path1_node_vector);
        else
            limit = index(1);  %to descard the subpath from the connection with infinite cost to the goal
        end
        
        available_nodes = 1;
    end
    if(verbose > 0)
        disp('Initial replanned_path_cost:')
        disp(replanned_path_cost);
    end
    
    change_j = 0;
    j = limit;
    
    if(verbose > 0)
        node_number = limit+idx;
        for n=1:length(replanned_path.connections)
            nodesPlot_vector = [nodesPlot_vector,replanned_path.connections(n).getParent]; %#ok<AGROW>
        end
    end
    
    while (j>0)
        
        if(informed>0)
            other_paths = reset_other_paths;
            if(flag_other_paths == 1) %if almost one replanned path has been found, the number of nodes of the subpath of the path2 to which the replaned path is connected are calculated
                n = length(confirmed_subpath_from_path2.connections);
                if(n == 0)
                    flag_other_paths = 0;
                end
                
                while(n > 0) 
                    if(n == 1)
                        flag_other_paths = 0;
                    end
                    if(isequal(path1_node_vector(j),confirmed_subpath_from_path2.connections(n).getParent)) %if the node analyzed is on the subpath2..(it happens only if informed == 2)
                        if(confirmed_connected2path_number<length(reset_other_paths))
                            other_paths = [reset_other_paths(1:confirmed_connected2path_number-1),confirmed_subpath_from_path2.getSubpathFromNode(confirmed_subpath_from_path2.connections(n).getParent),reset_other_paths(confirmed_connected2path_number+1:end)];
                        else
                            other_paths = [reset_other_paths(1:confirmed_connected2path_number-1),confirmed_subpath_from_path2.getSubpathFromNode(confirmed_subpath_from_path2.connections(n).getParent)];
                        end
                        n = 0;
                    else
                        other_paths = reset_other_paths; %if informed == 1 there are "n" while iterations unnecessary..you should correct it
                        n = n-1;
                    end
                end
            end
            
            [new_path,new_path_cost,solved,connected2PathNumber,subpathFromPath2] = PathSwitch(replanned_path,other_paths,path1_node_vector(j),lb,ub,max_distance,checker,metrics,opt_type,succ_node);
        else
            [new_path,new_path_cost,solved,connected2PathNumber,subpathFromPath2] = PathSwitch(current_path,other_paths,path1_node_vector(j),lb,ub,max_distance,checker,metrics,opt_type,succ_node);
        end
        
        path1_node_vector(j).setAnalyzed(1); %to set as ANALYZED the node just analyzed. In this way, it will not be analyzed again in this replanning procedure
        examined_nodes = [examined_nodes,path1_node_vector(j)]; %#ok<AGROW>
        
        if(verbose == 2)
            examined_nodes_plot = [examined_nodes(1,:).q];
        end

        if(verbose > 0)
            for d=1:length(nodesPlot_vector)
                if(nodesPlot_vector(d) == path1_node_vector(j))
                    node_number = d+idx;
                end
            end
        end
        
        if(solved==1)
            if(available_nodes==1) %calculating the cost of the replanned path found
                if(j>1)
                    subpath = subpath1.getSubpathToNode(path1_node_vector(j)); %the subpath that goes from the closest node to the robot conf, to the node from which the replanning has started
                    subpath_cost = subpath.cost;
                    path=Path([actual_node_conn,subpath.connections,new_path.connections]);    
                else
                subpath_cost = 0;
                path = Path([actual_node_conn,new_path.connections]); 
                end
            else
                actual_node_conn_cost = 0;
                subpath_cost = 0;
                path = new_path;
            end
    
            path_cost = actual_node_conn_cost+subpath_cost+new_path_cost;
            
            if(verbose > 0)
                disp('replanning from node')
                disp(node_number)
                disp('cost:')
                disp(path_cost)
                disp('replanned_path_cost:')
                disp(replanned_path_cost);
                disp('-------------------')
            end

            if(path_cost<replanned_path_cost) %if the cost of the new solution found is better than the cost of the best solution found so far
                
                if(nargout>5)
                    if(first_sol)
                        time_first_sol = toc;
                        first_sol = 0; %0 when the time of the first solution found has been already saved
                    end
                end
                
                previous_cost = replanned_path_cost;
                replanned_path = path;
                replanned_path_cost = path_cost;
                confirmed_connected2path_number = connected2PathNumber;
                confirmed_subpath_from_path2 = subpathFromPath2;
                success = 1;
                flag_other_paths = 1;
                
                if(verbose > 0)
                    nodesPlot_vector = [];
                    for n=1:length(replanned_path.connections)
                        nodesPlot_vector = [nodesPlot_vector,replanned_path.connections(n).getParent]; %#ok<AGROW>
                    end
                    number = j;
                end
                
                if(length(replanned_path_vector)<10) %the algorithm gives as output the vector of the best 10 solutions found, oredered by their cost
                    replanned_path_vector = [replanned_path,replanned_path_vector]; %#ok<AGROW>
                else
                    replanned_path_vector = [replanned_path,replanned_path_vector(1:end-1)];
                end
                
                if(informed==2 && available_nodes==1)   

                    if(verbose == 2) %To plot the graph at each iteration
                        PlotEnv %it is a script to plot the environment
                        if(~isempty(previous_joints))
                            plot3(previous_joints(1,:)',previous_joints(2,:)',previous_joints(3,:)','--c','LineWidth',0.5)
                            plot3(previous_joints(1,:)',previous_joints(2,:)',previous_joints(3,:)','*c','LineWidth',0.5)
                            plot3(old_node(1,:)',old_node(2,:)',old_node(3,:)','sc','LineWidth',2)
                        end
                        previous_joints = joints;
                        old_node = nodePlot;
                        hold off
                        disp(['Pause: press Enter, cost' num2str(replanned_path_cost)])
                        disp('-------------------')
                        pause
                    end

                    support = path1_node_vector(1:j-1); % the first j-1 nodes surely have not yet been analyzed
                    for r=1:length(new_path.connections)
                        if(new_path.connections(r).getParent.getAnalyzed == 0 && new_path.connections(r).getParent.getNonOptimal == 0) %Analyzed to check if they have been already analyzed (if 0 not not analyzed), NonOptimal to check if they are useful to improve the replanning solution (if 0, maybe they can improve the solution)
                            %the nodes of the new solution found are added to the set of the nodes to be analyzed
                            support = [support,new_path.connections(r).getParent]; %#ok<AGROW>  
                            change_j = change_j+1;
                        end
                    end
                    
                    path1_node_vector = support;
                    subpath1 = replanned_path.getSubpathFromNode(node);
                end
                    
            end
                
        else
            if(available_nodes==1 && verbose>0)
                
                disp('Replanning not possible/convenient from node number:')
                disp(node_number)

                if(verbose==2)  %To plot the graph at each iteration
                    PlotEnv
                    hold off
                    disp('Pause: press Enter')
                    disp('-------------------')
                    pause
                end
            end
        end
        
        if(toc>30) %stopping conditions
            j = 0;
        else
            if(toc>25 && cont>=5) 
                j = 0;    
            else
                if((previous_cost-replanned_path_cost) < 0.05*previous_cost) %note that replanned_path_cost is always lower than previous_cost
                    cont = cont+1;
                else
                    cont = 0;
                end
            end
        end  
        j = j-1;
        j = j+change_j;
        change_j = 0;
    end
    
    for i=1:length(examined_nodes)
        examined_nodes(i).setAnalyzed(0);
    end
    
    number_replanning = length(examined_nodes);
    
    if (success==1)
        if(verbose > 0)
            disp('replanned from node')
            disp(number+idx)
            disp('cost')
            disp(replanned_path_cost) 
        end
    else
        if (available_nodes==1)
            replanned_path = current_path; %path1
        else
            if(verbose > 0)
                disp('STOP');
            end
        end
    end
           
end
if(verbose == 2)
    hold on;
end

end