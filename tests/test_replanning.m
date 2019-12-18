%% Replaning feature
% Summary of example objective

clear all;close all;
connection_max_length=0.5;
% obstacle='sphere';
obstacle='cube';
% obstacle='snowman';
% obstacle='torus';
opt_type='full';

if strcmp(obstacle,'snowman')
    checker=Snowman3dCollisionChecker;
elseif strcmp(obstacle,'cube')
    checker=Cube3dCollisionChecker;
elseif strcmp(obstacle,'sphere')
    checker=Sphere3dCollisionChecker;
elseif strcmp(obstacle,'torus')
    checker=Torus3dCollisionChecker;
else
    error('invalid ostable')
end
checker.init;

figure('Position',[100 100 1200 600])
% subplot(121)
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

hold on
plot3(start_conf(1),start_conf(2),start_conf(3),'ob','MarkerFaceColor','b','MarkerSize',5)
plot3(goal_conf(1),goal_conf(2),goal_conf(3),'or','MarkerFaceColor','r','MarkerSize',5)

axis equal
xlabel('q1');
ylabel('q2');
zlabel('q3');

max_distance=0.5;

sampler = InformedSampler(start_conf,goal_conf,lb,ub);
solver1 = BirrtConnect(start_conf,goal_conf,max_distance,checker,sampler,metrics);

[success,path1]=solver1.solve;
if ~success
    fprintf('BiRRT Connect failed\n');
    return
end
path1=path1.resample(connection_max_length,metrics);

path_optimizer=PathLocalOptimizer(path1,opt_type,checker,metrics);
path_optimizer.solve;

fprintf('BiRRT Connect: cost=%f\n',path1.cost);

joints=path1.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','b','LineWidth',1)


solver2 = BirrtConnect(start_conf,goal_conf,max_distance,checker,sampler,metrics);
[success,path2]=solver2.solve;
if ~success
    fprintf('BiRRT Connect failed\n');
    return
end
path2=path2.resample(connection_max_length,metrics);


path_optimizer=PathLocalOptimizer(path2,opt_type,checker,metrics);
path_optimizer.solve;
fprintf('BiRRT Connect: cost=%f\n',path2.cost);


joints=path2.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','r','LineWidth',1)


%% 

idx_replan=round(length(path1.connections)*0.5);
path1_node=path1.connections(:,idx_replan).getChild;
path2_node=path2.findCloserNode(path1_node.q);

plot3(path1_node.q(1),path1_node.q(2),path1_node.q(3),'ok')
plot3(path2_node.q(1),path2_node.q(2),path2_node.q(3),'dk')

% TODO occorre cancellare le connessioni esistenti per mantenere la
% struttura ad albero (un solo padre o un solo figlio, e root senza connessioni). Valutare come e se
% cambiarlo

subpath1=path1.getSubpathToNode(path1_node).connections(1:end-1);
subpath2=path2.getSubpathFromNode(path2_node).connections(2:end);

path1_node.child_connections.delete
path1_node.parent_connections.delete
if ~isempty(path2_node.child_connections)
    path2_node.child_connections.delete
end
if ~isempty(path2_node.parent_connections)
    path2_node.parent_connections.delete
end

% si potrebbe dire al replanning sampling un costo massimo di ricerca    
replan_sampler = InformedSampler(path1_node.q,path2_node.q,lb,ub);

solver_replannig = BirrtExtend(path1_node,path2_node,max_distance,checker,replan_sampler,metrics);

[success,replanned_path]=solver_replannig.solve;
if ~success
    fprintf('BiRRT Connect failed\n');
    return
end

replanned_path.verboseDebug(false);
path_optimizer=PathLocalOptimizer(replanned_path,opt_type,checker,metrics);
path_optimizer.solve;
joints=replanned_path.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','--m','LineWidth',2)

if isempty(subpath1)
    joint_subpath1=[];
else
    conn_cost=metrics.cost(subpath1(end).getChild,path1_node);
    joint_subpath1=Connection(subpath1(end).getChild,path1_node,conn_cost);
end
if isempty(subpath2)
    joint_subpath2=[];
else
    conn_cost=metrics.cost(path2_node,subpath2(1).getParent);
    joint_subpath2=Connection(path2_node,subpath2(1).getParent,conn_cost);
end
new_path=Path([subpath1 joint_subpath1 replanned_path.connections joint_subpath2 subpath2]);
joints=new_path.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','-m','LineWidth',1)
