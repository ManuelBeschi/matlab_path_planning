function [replanned_path,replanned_path_cost,success] = OnlineReplanning(current_path,other_paths,q,lb,ub,max_distance,checker,metrics,opt_type,succ_node)

replanned_path = [];
replanned_path_cost = inf;
success = 0;

index = [];
idx = findConnection(current_path,q);
disp('idx:')
disp(idx)

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
        
        path1 = current_path.getSubpathFromNode(node);
        replanned_path_cost = path1.cost;

        available_nodes = 0;
        limit = 1;
        path1_node_vector = node;
    else
        node = child; 
        actual_node_conn_cost = metrics.cost(actual_node,node);
        actual_node_conn = Connection(actual_node,node,actual_node_conn_cost);
        subpath1 = current_path.getSubpathFromNode(node);
        subpath1_conn = subpath1.connections;
      
        path1 = Path([actual_node_conn,subpath1_conn]);
        replanned_path_cost = path1.cost;

        
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
    disp('Initial replanned_path_cost:')
    disp(replanned_path_cost); 
    
    for j=limit:-1:1
        [new_path,new_path_cost,solved] = PathSwitch(current_path,other_paths,path1_node_vector(j),lb,ub,max_distance,checker,metrics,opt_type,succ_node);
        
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
            
            disp('replanning from node')
            disp(j+idx)
            disp('cost:')
            disp(path_cost)
            disp('replanned_path_cost:')
            disp(replanned_path_cost);            
            disp('-----------')

            if(path_cost<replanned_path_cost)
                replanned_path = path;
                replanned_path_cost = path_cost;
                success = 1;
                number = j;    
            end
                
            else
                if(available_nodes==1)
                    disp('Replanning not possible/convenient with the node number:')
                    disp(j+idx)
                end
         end
    end
    
    if (success==1)
        disp('replanned from node')
        disp(number+idx)
        disp('cost')
        disp(replanned_path_cost)
    else
        if (available_nodes==1)
            replanned_path = path1;
        else
            disp('STOP');
        end
    end
           
end
end

%VALUTA LA POSSIBILITÀ DI RIPIANIFICARE ANCHE A PARTIRE DA ACTUAL NODE
%SEMPRE, ANCHE SE IL COSTO DELL'AUTTALE CONNESSIONE NON È INF. SE SI EVITA
%SI RISPARMIA CALCOLO E TEMPO