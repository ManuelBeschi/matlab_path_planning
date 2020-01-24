function [new_path,success] = PathSwitch(current_path,other_paths,q,lb,ub,max_distance,checker,metrics,opt_type,succ_node)
% [new_path,success] = PathSwitch(current_path,other_paths,q,lb,ub,max_distance,checker,metrics,opt_type,succ_node)
% OUTPUT:
%> new_path: the calculated path that starts from the current_path node with joints poistion q and moves to another path of other_paths
%> success: 1 if the path switch is possible, 0 otherwise.
% INPUT:
%> current_path: the path currently traveled
%> other_paths: vector of the other available paths
%> q: joints poistion from which the path switch starts
%> lb, ub: lower and upper joints bounds
%> max_distance: the max distance between node admissible for the BirrtExtend algorithm
%> checker: the collision checker
%> metrics: the metric used to calculate the connections cost
%> opt_type ?????????????????????????????????
%> succ_node: if 1, the PathSwitch function looks for the best node of
% all the other paths to connect to, oherwise it considers, for any other
% path, the closest node.


%NB: Dovrai chiedere di fare una funzione in path che se gli dai q ti crea un
%nodo in quella posizione, è il nodo che usi per definire l'inizio dello switch del path
% Individuo la prima porzione del percorso, che è il tratto che ho percorso
% fino ad ora
new_path = [];
success = 0;

path1_node = current_path.findCloserNode(q); %per ora fai così, è temporaneo %_____
path_cost = inf;
switch2path = 0;
switch2node = 0;

for j = 1:length(other_paths)
    
    path2 = other_paths(j);
    
    if(succ_node == 1)
        path2_conn = path2.connections;
        path2_node_vector = path2_conn(1).getParent;
        for t=1:length(path2_conn)-1  %non mi serve il nodo GOAL
            path2_node_vector = [path2_node_vector,path2_conn(t).getChild]; %#ok<AGROW>
        end
    else     
        path2_node_vector = path2.findCloserNode(path1_node.q);
    end
    for k=1:length(path2_node_vector)
 
            path2_node = path2_node_vector(k);

            path2_node2goal = path2.getSubpathFromNode(path2_node);
            subpath2 = path2_node2goal.connections;
            
            path1_node_fake = Node(path1_node.q);
            path2_node_fake = Node(path2_node.q);
            
            replan_sampler = InformedSampler(path1_node_fake.q,path2_node_fake.q,lb,ub);
%             if path_cost<inf
%             sampler.setCost(path_cost)
%             end
            solver_replannig = BirrtExtend(path1_node_fake,path2_node_fake,max_distance,checker,replan_sampler,metrics);
            [~,replanned_path] = solver_replannig.solve; %check if solved
            replanned_path.verboseDebug(false);
            path_optimizer=PathLocalOptimizer(replanned_path,opt_type,checker,metrics);
            path_optimizer.solve; 
            connecting_path = path_optimizer.path.connections;
            
            node1 = connecting_path(1).getChild;
            node2 = connecting_path(end).getParent;
            conn1_cost = metrics.cost(path1_node,node1);
            conn2_cost = metrics.cost(node2,path2_node);
            conn1 = Connection(path1_node,node1,conn1_cost);
            conn2 = Connection(node2,path2_node,conn2_cost);
            
            connecting_path = [conn1,connecting_path(2:end-1),conn2];

            path1_node_fake.delete;
            path2_node_fake.delete;
    
            conn_cost = 0;
            for i=1:length(subpath2)
                conn_cost=conn_cost+metrics.cost(subpath2(i).getParent,subpath2(i).getChild);
            end
            for i=1:length(connecting_path)
                    conn_cost=conn_cost+metrics.cost(connecting_path(i).getParent,connecting_path(i).getChild);
            end
    
            if(conn_cost<=path_cost && ~isempty(subpath2))
                    new_path=Path([connecting_path subpath2]);
                    switch2path = j;
                    switch2node = k;
                    path_cost = conn_cost;
                    success = 1;
            end 

    end

end
disp('Connection to the path number:')
disp(switch2path)
if(succ_node == 1)
    disp('To the node number:')
    disp(switch2node)
end
end


