function [replanned_path,replanned_path_cost,success,replanned_path_vector] = InformedOnlineReplanning(current_path,other_paths,q,lb,ub,max_distance,checker,metrics,opt_type,succ_node,informed,verbose)
%function [replanned_path,replanned_path_cost,success,replanned_path_vector] = InformedOnlineReplanning(current_path,other_paths,q,lb,ub,max_distance,checker,metrics,opt_type,succ_node,informed,verbose)
% OUTPUT:
%> replanned_path: the calculated path that minimizes the cost
%>replanned_path_cost: the cost of the new planned path
%> success: 1 if the algorithm succeeds, 0 otherwise.
%>replanned_path_vector: the vector containing the 10 best paths found.
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
%>informed: if 0, the algorithm is not informed (always current_path in
%PathSwitch). If 1, the algorithm is informed (replanned_path in PathSwitch)
%but the node_vector analyzed is not updated with the new nodes of the new
%path. If 2, the node vector is updated with the new nodes added.
%>verbose: if 0, no comments displayed, if 1 comment displayed, if 2
%comment and updated graph for each iteration displayed. Red square: actual
%node analyzed; cyan square: last starting node for replanning; cyan path:
%last replanned path

replanned_path = [];
replanned_path_vector = [];
replanned_path_cost = inf;
success = 0;

index = [];
idx = current_path.findConnection(q);

if(verbose > 0)
    previous_joints = [];
    old_node = [];
    nodesPlot_vector = [];
    node_number = 0;
    disp('idx:')
    disp(idx)
end

if(idx>0)
    
    path1_node_vector = [];
    parent = current_path.connections(idx).getParent;
    child = current_path.connections(idx).getChild;
    
    if(norm(parent.q-q,2)<1e-6)
        actual_node = parent;
    elseif(norm(child.q-q,2)<1e-6)
        actual_node = child;
    else
        actual_node = Node(q);
    end
    
    if(current_path.connections(idx).getCost == inf)
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
        
        replanned_path = current_path.getSubpathFromNode(node);  %inizializzo replanned path con la porzione di current_path che devo percorrere, poi lo aggiornerò volta per volta con il migliore trovato
        replanned_path_cost = replanned_path.cost;

        available_nodes = 0;
        limit = 1;
        path1_node_vector = node;
    else
        node = child; 
        actual_node_conn_cost = metrics.cost(actual_node,node);
        actual_node_conn = Connection(actual_node,node,actual_node_conn_cost);
        subpath1 = current_path.getSubpathFromNode(node);
        subpath1_conn = subpath1.connections;

        replanned_path = Path([actual_node_conn,subpath1_conn]);
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
            limit = index(1);  %escludo dalla connessione con costo infinito in poi
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
    
    while (j>0)  %il FOR non mi permette di modificare l'indice di iterazione in corso
        j = j+change_j;
        change_j = 0;
        if(informed>0)
            [new_path,new_path_cost,solved] = PathSwitch(replanned_path,other_paths,path1_node_vector(j),lb,ub,max_distance,checker,metrics,opt_type,succ_node);
        else
            [new_path,new_path_cost,solved] = PathSwitch(current_path,other_paths,path1_node_vector(j),lb,ub,max_distance,checker,metrics,opt_type,succ_node);
        end  
        
        if(verbose > 0)
            for d=1:length(nodesPlot_vector)
                if(nodesPlot_vector(d) == path1_node_vector(j))
                    node_number = d+idx;
                end
            end
        end
        
        if(solved==1)
            if(available_nodes==1)
                if(j>1)
                    subpath = subpath1.getSubpathToNode(path1_node_vector(j)); %path che va dal nodo più vicino a me fino al nodo da cui inizio lo switch
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
                disp('-----------')
            end

            if(path_cost<replanned_path_cost)
                replanned_path = path;
                replanned_path_cost = path_cost;
                success = 1;
                
                if(verbose > 0)
                    nodesPlot_vector = [];
                    for n=1:length(replanned_path.connections)
                        nodesPlot_vector = [nodesPlot_vector,replanned_path.connections(n).getParent]; %#ok<AGROW>
                    end
                    number = j;
                end
                
                if(length(replanned_path_vector)<10) %li inserisco in modo che siano già ordinati
                    replanned_path_vector = [replanned_path,replanned_path_vector]; %#ok<AGROW>
                else
                    replanned_path_vector = [replanned_path,replanned_path_vector(1:end-1)];
                end
                
                if(informed==2 && available_nodes==1)                
                    if(verbose == 2)
                        path1 = current_path;
                        path2 = other_paths(1);
                        path3 = other_paths(2);
                        path1_nodes = path1.getWaypoints;
                        path2_nodes = path2.getWaypoints;
                        path3_nodes = path3.getWaypoints;
                        joints=replanned_path.getWaypoints;
                        nodePlot = path1_node_vector(j).q;
                        
                        clf
                        
                        hold on
                        obstacle='snowman';
                        checker=Snowman3dCollisionChecker;
                        checker.init;
                        checker.plot
                        checker.plot
                        metrics=Metrics;
                        view(135,0)
                        lb=-pi*ones(3,1);
                        ub=pi*ones(3,1);
                        if strcmp(obstacle,'torus')
                            start_conf = [0 1.1 0]';
                            goal_conf = [0.6 -0.7 0]';
                        else
                            start_conf = [0.0 -1.1 0]';
                            goal_conf =  [0.0 1.1 0]';
                        end
                        plot3(start_conf(1),start_conf(2),start_conf(3),'sy','MarkerFaceColor','b','MarkerSize',5)
                        plot3(goal_conf(1),goal_conf(2),goal_conf(3),'oy','MarkerFaceColor','r','MarkerSize',5)
                        axis equal
                        xlabel('q1');
                        ylabel('q2');
                        zlabel('q3');
                        plot3(path1_nodes(1,:)',path1_nodes(2,:)',path1_nodes(3,:)','--*b','LineWidth',0.5)
                        plot3(path2_nodes(1,:)',path2_nodes(2,:)',path2_nodes(3,:)','--*r','LineWidth',0.5)
                        plot3(path3_nodes(1,:)',path3_nodes(2,:)',path3_nodes(3,:)','--*g','LineWidth',0.5)
                        plot3(joints(1,:)',joints(2,:)',joints(3,:)','-y','LineWidth',1)
                        plot3(joints(1,:)',joints(2,:)',joints(3,:)','oy','LineWidth',1)
                        plot3(q(1,:)',q(2,:)',q(3,:)','*c','LineWidth',2)
                        plot3(nodePlot(1,:)',nodePlot(2,:)',nodePlot(3,:)','sr','LineWidth',2)
                        if(~isempty(previous_joints))
                            plot3(previous_joints(1,:)',previous_joints(2,:)',previous_joints(3,:)','--c','LineWidth',0.5)
                            plot3(previous_joints(1,:)',previous_joints(2,:)',previous_joints(3,:)','*c','LineWidth',0.5)
                            plot3(old_node(1,:)',old_node(2,:)',old_node(3,:)','sc','LineWidth',2)
                        end
                        previous_joints = joints;
                        old_node = nodePlot;
                        hold off
                        disp('Pause: press Enter')
                        disp('-------------------')
                        pause
                    end
                    
                    path1_node_vector = path1_node_vector(1:j-1);
                    for i=2:length(new_path.connections)  %il nodo iniziale l'ho appena analizzato, non analizzo di nuovo
                            path1_node_vector = [path1_node_vector,new_path.connections(i).getParent]; %#ok<AGROW>
                    end
                    
                    subpath1 = replanned_path.getSubpathFromNode(node);
                    change_j = length(new_path.connections)-1; %corrisponde al numero di nodi aggiunti dal newpath
                end
                    
            end
                
        else
            if(available_nodes==1 && verbose>0)
                disp('Replanning not possible/convenient from node number:')
                disp(node_number)

                if(verbose==2)
                    path1 = current_path;
                    path2 = other_paths(1);
                    path3 = other_paths(2);
                    path1_nodes = path1.getWaypoints;
                    path2_nodes = path2.getWaypoints;
                    path3_nodes = path3.getWaypoints;
                    joints=replanned_path.getWaypoints;
                    nodePlot = path1_node_vector(j).q;
                    
                    clf
                    
                    hold on
                    obstacle='snowman';
                    checker=Snowman3dCollisionChecker;
                    checker.init;
                    checker.plot
                    checker.plot
                    metrics=Metrics;
                    view(135,0)
                    lb=-pi*ones(3,1);
                    ub=pi*ones(3,1);
                    if strcmp(obstacle,'torus')
                        start_conf = [0 1.1 0]';
                        goal_conf = [0.6 -0.7 0]';
                    else
                        start_conf = [0.0 -1.1 0]';
                        goal_conf =  [0.0 1.1 0]';
                    end
                    plot3(start_conf(1),start_conf(2),start_conf(3),'sy','MarkerFaceColor','b','MarkerSize',5)
                    plot3(goal_conf(1),goal_conf(2),goal_conf(3),'oy','MarkerFaceColor','r','MarkerSize',5)
                    axis equal
                    xlabel('q1');
                    ylabel('q2');
                    zlabel('q3');
                    plot3(path1_nodes(1,:)',path1_nodes(2,:)',path1_nodes(3,:)','--*b','LineWidth',0.5)
                    plot3(path2_nodes(1,:)',path2_nodes(2,:)',path2_nodes(3,:)','--*r','LineWidth',0.5)
                    plot3(path3_nodes(1,:)',path3_nodes(2,:)',path3_nodes(3,:)','--*g','LineWidth',0.5)
                    plot3(joints(1,:)',joints(2,:)',joints(3,:)','-y','LineWidth',1)
                    plot3(joints(1,:)',joints(2,:)',joints(3,:)','oy','LineWidth',1)
                    plot3(q(1,:)',q(2,:)',q(3,:)','*c','LineWidth',2)
                    plot3(nodePlot(1,:)',nodePlot(2,:)',nodePlot(3,:)','sr','LineWidth',2)
                    
                    hold off
                    disp('Pause: press Enter')
                    disp('-------------------')
                    pause
                end
            end
        end
        j = j-1;
    end
    
    if (success==1)
        if(verbose > 0)
            disp('replanned from node')
            disp(number+idx)
            disp('cost')
            disp(replanned_path_cost) 
        end
    else
        if (available_nodes==1)
            replanned_path = path1;
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


%VALUTA LA POSSIBILITÀ DI RIPIANIFICARE ANCHE A PARTIRE DA ACTUAL NODE
%SEMPRE, ANCHE SE IL COSTO DELL'AUTTALE CONNESSIONE NON È INF. SE SI EVITA
%SI RISPARMIA CALCOLO E TEMPO