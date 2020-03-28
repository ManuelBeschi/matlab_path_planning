function [replanned_path,replanned_path_cost,success,replanned_path_vector,number_replanning] = InformedOnlineReplanning(current_path,other_paths,q,lb,ub,max_distance,checker,metrics,opt_type,succ_node,informed,verbose)
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
previous_cost = current_path.cost;
success = 0;
flag_other_paths = 0;
examined_nodes = [];
cont = 0;
number_replanning = 0;

index = [];
idx = current_path.findConnection(q);
admissible_current_path = [];

%verbose =2; %ELIMINA

if(verbose > 0)
    previous_joints = [];
    old_node = [];
    nodesPlot_vector = [];
    node_number = 0;
    disp('idx:')
    disp(idx)
    
    if(verbose == 2)
        examined_nodes_plot = [];
    end
end

%verbose = 0; %ELIMINA

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
              
    z = length(current_path.connections);  %Ora individuo la parte di current_path percorribile e dunque da non scartare
    while(z>idx)  %arrivo massimo alla connessione successiva a quella attuale
        if(current_path.connections(z).getCost == inf)
            if(z == length(current_path.connections))
                admissible_current_path = [];
            else
                admissible_current_path = current_path.getSubpathFromNode(current_path.connections(z).getChild);
            end
            z = 0;
        else
            z = z-1;
        end
    end
    
    if(isa(admissible_current_path,'Path'))
        other_paths = [admissible_current_path,other_paths];  %aggiungo ai possibili path il tratto finale con costo non infinito di current_path
    end
    
    reset_other_paths = other_paths; %serve piu avantu per tornare al vettore completo 
    
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
    
    %verbose = 2; %ELIMINA
    if(verbose > 0)
        node_number = limit+idx;
        for n=1:length(replanned_path.connections)
            nodesPlot_vector = [nodesPlot_vector,replanned_path.connections(n).getParent]; %#ok<AGROW>
        end
    end
    %verbose = 0; %ELIMINA
    
    while (j>0)  %il FOR non mi permette di modificare l'indice di iterazione in corso
        j = j+change_j;
        change_j = 0;
        
        if(informed>0)
            other_paths = reset_other_paths;
            if(flag_other_paths == 1) %flag_other_paths == 1 per essere sicuro che almeno una volta son riuscito a ripianificare per cui mi sono collegato ad un path..altrimenti questi calcoli non servono
                n = length(confirmed_subpath_from_path2.connections);
                while(n > 0)
                    if(n == 1)
                        flag_other_paths = 0;
                    end
                    if(isequal(path1_node_vector(j),confirmed_subpath_from_path2.connections(n).getParent))
                        if(confirmed_connected2path_number<length(reset_other_paths))
                            other_paths = [reset_other_paths(1:confirmed_connected2path_number-1),confirmed_subpath_from_path2.getSubpathFromNode(confirmed_subpath_from_path2.connections(n).getParent),reset_other_paths(confirmed_connected2path_number+1:end)];
                        else
                            other_paths = [reset_other_paths(1:confirmed_connected2path_number-1),confirmed_subpath_from_path2.getSubpathFromNode(confirmed_subpath_from_path2.connections(n).getParent)]; %modificato confirmed_subpath_from_path2 con confirmed_subpath_from_path2.getSubpathFromNode(confirmed_subpath_from_path2.connections(n).getParent) qui e la riga sopra
                        end
                        n = 0;
                    else
                        other_paths = reset_other_paths;
                        n = n-1;
                    end
                end
            end
            
            [new_path,new_path_cost,solved,connected2PathNumber,subpathFromPath2] = PathSwitch(replanned_path,other_paths,path1_node_vector(j),lb,ub,max_distance,checker,metrics,opt_type,succ_node);
        else
            [new_path,new_path_cost,solved,connected2PathNumber,subpathFromPath2] = PathSwitch(current_path,other_paths,path1_node_vector(j),lb,ub,max_distance,checker,metrics,opt_type,succ_node);
        end
        
        path1_node_vector(j).setAnalyzed(1); %segnalo che il nodo è stato utilizzato per il replanning così non lo riutilizzero in futuro   ANALYZED FALSO NO ANALIZZATO
        examined_nodes = [examined_nodes,path1_node_vector(j)]; %#ok<AGROW>
        
        %verbose = 2; %ELIMINA
        
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
        
        %verbose = 0; %ELIMINA
        
        if(solved==1)
            number_replanning=number_replanning+1;
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
                disp('-------------------')
            end

            if(path_cost<replanned_path_cost)
                previous_cost = replanned_path_cost;
                replanned_path = path;
                replanned_path_cost = path_cost;
                confirmed_connected2path_number = connected2PathNumber;
                confirmed_subpath_from_path2 = subpathFromPath2;
                success = 1;
                flag_other_paths = 1;
                
                %verbose = 2; %ELIMINA
                if(verbose > 0)
                    nodesPlot_vector = [];
                    for n=1:length(replanned_path.connections)
                        nodesPlot_vector = [nodesPlot_vector,replanned_path.connections(n).getParent]; %#ok<AGROW>
                    end
                    number = j;
                end
                %verbose = 0; %ELIMINA
                
                if(length(replanned_path_vector)<10) %li inserisco in modo che siano già ordinati
                    replanned_path_vector = [replanned_path,replanned_path_vector]; %#ok<AGROW>
                else
                    replanned_path_vector = [replanned_path,replanned_path_vector(1:end-1)];
                end
                
                if(informed==2 && available_nodes==1)   
%                     if(toc>40) %ELIMINA
%                         verbose = 2;
%                     else
%                         verbose = 0;
%                     end
                    if(verbose == 2) %To plot the graph at each iteration
                        PlotEnv
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

                    support = path1_node_vector(1:j-1); %i nodi fino a j-1 sicuramente non sono stati ancora analizzati
                    for r=1:length(new_path.connections)
                        if(new_path.connections(r).getParent.getAnalyzed == 0 && new_path.connections(r).getParent.getNonOptimal == 0)
                            support = [support,new_path.connections(r).getParent]; %#ok<AGROW>
                            change_j = change_j+1;
                        end
                    end
                    path1_node_vector = support;
                    
                    subpath1 = replanned_path.getSubpathFromNode(node);
%                    change_j = length(new_path.connections)-1; %corrisponde al numero di nodi aggiunti dal newpath
                end
                    
            end
                
        else
            if(available_nodes==1 && verbose>0)
                
%                 if(toc>40) %ELIMINA
%                     verbose = 2;
%                 else
%                     verbose = 0;
%                 end
                
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
        
        if(toc>30)
            j = 0;
        else
            if(toc>25 && cont>=5) %definisco condizione limite per evitare soluzioni troppo costose in termini di tempo
                j = 0;    
            else
                if((previous_cost-replanned_path_cost) < 0.05*previous_cost)
                    cont = cont+1;
                else
                    cont = 0;
                end
            end
        end  
        j = j-1;
    end
    
    for i=1:length(examined_nodes)
        examined_nodes(i).setAnalyzed(0);   %eventualmente, questa cosa la puoi fare mentre il robot sta percorrendo il path che hai trovato così risparmi tempo
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


%VALUTA LA POSSIBILITÀ DI RIPIANIFICARE ANCHE A PARTIRE DA ACTUAL NODE
%SEMPRE, ANCHE SE IL COSTO DELL'AUTTALE CONNESSIONE NON È INF. SE SI EVITA
%SI RISPARMIA CALCOLO E TEMPO