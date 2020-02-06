function [replanned_path,replanned_path_cost,success,replanned_path_vector] = InformedOnlineReplanning(current_path,other_paths,q,lb,ub,max_distance,checker,metrics,opt_type,succ_node,informed)
%function [replanned_path,replanned_path_cost,success,replanned_path_vector] = InformedOnlineReplanning(current_path,other_paths,q,lb,ub,max_distance,checker,metrics,opt_type,succ_node,informed)
% OUTPUT:
%> replanned_path: the calculated path that minimizes the cost
%>replanned_path_cost: the cost of the new planned path
%> success: 1 if the algorithm succeeds, 0 otherwise.
%>replanned_path_vector: the vector containing the 10 best paths found.
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
%>informed: if 0, the algorithm is not informed (always current_path in
%PathSwitch). If 1, the algorithm is informed (replanned_path in PathSwitch)
%but the node_vector analyzed is not updated with the new nodes of the new
%path. If 2, the node vector is updated with the new nodes added.

replanned_path = [];
replanned_path_vector = [];
replanned_path_cost = inf;
success = 0;
verbose = 1;


index = [];
idx = current_path.findConnection(q);
if(verbose)
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
    if(verbose)
        disp('Initial replanned_path_cost:')
        disp(replanned_path_cost);
    end
    
    change_j = 0;
    j = limit;
    
    while (j>0)  %il FOR non mi permette di modificare l'indice di iterazione in corso

        j = j+change_j;
        change_j = 0;
        if(informed>0)
            [new_path,new_path_cost,solved] = PathSwitch(replanned_path,other_paths,path1_node_vector(j),lb,ub,max_distance,checker,metrics,opt_type,succ_node);
        else
            [new_path,new_path_cost,solved] = PathSwitch(current_path,other_paths,path1_node_vector(j),lb,ub,max_distance,checker,metrics,opt_type,succ_node);
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
            
            if(verbose)
                disp('replanning from node')
                disp(j+idx)
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
                
                if(verbose)
                    number = j;
                end
                
                if(length(replanned_path_vector)<10) %li inserisco in modo che siano già ordinati
                    replanned_path_vector = [replanned_path,replanned_path_vector]; %#ok<AGROW>
                else
                    replanned_path_vector = [replanned_path,replanned_path_vector(1:end-1)];
                end
                
                if(informed==2 && available_nodes==1)
                    path1_node_vector = path1_node_vector(1:j-1);
                    for i=2:length(new_path.connections)  %il nodo iniziale l'ho appena analizzato, non analizzo di nuovo
                            path1_node_vector = [path1_node_vector,new_path.connections(i).getParent]; %#ok<AGROW>
                    end

                    subpath1 = replanned_path.getSubpathFromNode(node);
                    change_j = length(new_path.connections)-1; %corrisponde al numero di nodi aggiunti dal newpath
                end
                    
            end
                
            else
                if(available_nodes==1 && verbose)
                    disp('Replanning not possible/convenient from node number:')
                    disp(j+idx)
                end
        end
         j = j-1;
    end
    
    if (success==1)
        if(verbose)
            disp('replanned from node') %#ok<*UNRCH>
            disp(number+idx)
            disp('cost')
            disp(replanned_path_cost)
        end
    else
        if (available_nodes==1)
            replanned_path = path1;
        else
            if(verbose)
                disp('STOP');
            end
        end
    end
           
end
end

%VALUTA LA POSSIBILITÀ DI RIPIANIFICARE ANCHE A PARTIRE DA ACTUAL NODE
%SEMPRE, ANCHE SE IL COSTO DELL'AUTTALE CONNESSIONE NON È INF. SE SI EVITA
%SI RISPARMIA CALCOLO E TEMPO