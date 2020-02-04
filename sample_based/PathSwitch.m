function [new_path,path_cost,success] = PathSwitch(current_path,other_paths,node,lb,ub,max_distance,checker,metrics,opt_type,succ_node)
% [new_path,success] = PathSwitch(current_path,other_paths,q,lb,ub,max_distance,checker,metrics,opt_type,succ_node)
% OUTPUT:
%> new_path: the calculated path that starts from the current_path node with joints poistion q and moves to another path of other_paths
%>path_cost: the cost of the new planned path
%> success: 1 if the path switch is possible, 0 otherwise.
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


% Individuo la prima porzione del percorso, che è il tratto che ho percorso
% fino ad ora
new_path = [];
success = 0;
switch2path = 0;
switch2node = 0;

path1_node = node;
path1_node2goal = current_path.getSubpathFromNode(path1_node);
subpath1_cost = path1_node2goal.cost;

%path_cost = inf;
path_cost = subpath1_cost;

for j = 1:length(other_paths)
    
    path2 = other_paths(j);
    
    if(succ_node == 1)
        path2_conn = path2.connections;
        path2_node_vector = [];
        for t=1:length(path2_conn)  %non mi serve il nodo GOAL
            path2_node_vector = [path2_node_vector,path2_conn(t).getParent]; %#ok<AGROW>
        end
    else     
        path2_node_vector = path2.findCloserNode(path1_node.q);
    end
    
    for k=1:length(path2_node_vector)
 
            path2_node = path2_node_vector(k);

            path2_node2goal = path2.getSubpathFromNode(path2_node);
            subpath2 = path2_node2goal.connections;
            subpath2_cost = path2_node2goal.cost;
            
            diff_subpath_cost = path_cost-subpath2_cost;  %margine di costo concesso a connecting_path VALUTA ALTERNATIVA(PEGGIORE) CON SUBPATH1_COST AL POSTO DI PATH_COST CHE SI AGGIORNA
            distance_path_node = norm(path1_node.q-path2_node.q,2);
            
            if (distance_path_node<diff_subpath_cost) %se il costo della porzione rimanente di path1 è maggiore di quella di path2 ha senso verificare se muoversi verso path2 è vantaggioso in termini di costo
                                                                             %se la distanza fra i due nodi è maggiore in termini di costo di quello concesso al connecting path devo escludere, meglio di una retta non posso fare..VALE SOLO SE LA METRICA E' LA DISTANZA               
                path1_node_fake = Node(path1_node.q);
                path2_node_fake = Node(path2_node.q);
            
                replan_sampler = InformedSampler(path1_node_fake.q,path2_node_fake.q,lb,ub,diff_subpath_cost); %uso l'ellisse con dimensione dettata dal costo massimo che il connecting path potrà avere, oltre a questo valore subpath1 conviene di più
                %replan_sampler.setCost(diff_subpath_cost)

                solver_replannig = BirrtExtend(path1_node_fake,path2_node_fake,max_distance,checker,replan_sampler,metrics);
                [solved,replanned_path] = solver_replannig.solve;
                
                if(solved) %se sono riuscito a pianificare un connecting path..
                    replanned_path.verboseDebug(false);
                    path_optimizer=PathLocalOptimizer(replanned_path,opt_type,checker,metrics);
                    path_optimizer.solve; 
                    connecting_path = path_optimizer.path.connections;
    
                    conn_cost = subpath2_cost;
                    for i=1:length(connecting_path)
                        conn_cost=conn_cost+metrics.cost(connecting_path(i).getParent,connecting_path(i).getChild);
                    end
                    
%                     disp('path number, node number:')
%                     disp(j);
%                     disp(k);
%                     disp('cost:')
%                     disp(conn_cost)
                    
                    if(conn_cost<path_cost && conn_cost<subpath1_cost && ~isempty(subpath2)) %%DUBBIO %verifico sia che il costo del connecting path sia compatibile con il limite dato da diff_subpath_cost (altrimenti conviene subpath1) e verifico che il costo di questo connecting path sia minore di quelli precedenti
                                               
                        node1 = connecting_path(1).getChild;  %FACCIO QUESTI COLLEGAMENTI SOLO SE IL PATH È CONVENIENTE, ALTRIMENTI È INUTILE
                        node2 = connecting_path(end).getParent;
                        conn1_cost = metrics.cost(path1_node,node1);
                        conn2_cost = metrics.cost(node2,path2_node);
                        conn1 = Connection(path1_node,node1,conn1_cost);
                        conn2 = Connection(node2,path2_node,conn2_cost);
            
                        connecting_path = [conn1,connecting_path(2:end-1),conn2];

                        path1_node_fake.delete;
                        path2_node_fake.delete;
                        
                        new_path=Path([connecting_path subpath2]);
                        switch2path = j;
                        switch2node = k;
                        path_cost = conn_cost;
                        success = 1;
%                       disp('Connection  POSSIBLE')
%                       disp('-------------------------------------------');
                    end 
                else
%                     disp('Connection NOT POSSIBLE to the node number:')
%                     disp(k);
%                     disp('of the path number:')
%                     disp(j);
%                     disp('-------------------------------------------');
                end
            else
%                 disp('It is better subpath1 than subpath2')
%                 disp('path number:')
%                 disp(j);
%                 disp('node number:')
%                 disp(k);
%                 disp(diff_subpath_cost)
%                 disp(distance_path_node)
%                 disp('-------------------------------------------');
            end
    end

end
% disp('Connection to the path number:')
% disp(switch2path)
% disp('-------------------------------------------');
%if(succ_node == 1)
%     disp('To the node number:')
%     disp(switch2node)
%     disp('-------------------------------------------');
%end
% disp('cost:')
% disp(path_cost)
end
%NB: SEGNALA CHE TALVOLTA QUALCH NODO DELLA RETE NON È CORRETTAMENTE
%CONNESSO ALLA RETE (es non ha parent_connections)

