clearvars; close all; clc;

connection_max_length=0.5;
%obstacle='sphere';
%obstacle='cube';
obstacle='snowman';
%obstacle='torus';
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
plot3(start_conf(1),start_conf(2),start_conf(3),'sy','MarkerFaceColor','b','MarkerSize',5)
plot3(goal_conf(1),goal_conf(2),goal_conf(3),'oy','MarkerFaceColor','r','MarkerSize',5)

axis equal
xlabel('q1');
ylabel('q2');
zlabel('q3');

max_distance=0.5;


%% 1° Path
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

path1.connections(end).setCost(3.7);

fprintf('BiRRT Connect: cost=%f\n',path1.cost);

joints=path1.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','--b','LineWidth',1)

%% 2° Path
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
plot3(joints(1,:)',joints(2,:)',joints(3,:)','--r','LineWidth',1)

%% 3° Path
solver3 = BirrtConnect(start_conf,goal_conf,max_distance,checker,sampler,metrics);
[success,path3]=solver3.solve;
if ~success
    fprintf('BiRRT Connect failed\n');
    return
end
path3=path3.resample(connection_max_length,metrics);


path_optimizer=PathLocalOptimizer(path3,opt_type,checker,metrics);
path_optimizer.solve;
fprintf('BiRRT Connect: cost=%f\n',path3.cost);


joints=path3.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','--g','LineWidth',1)

%% 4° Path
solver4 = BirrtConnect(start_conf,goal_conf,max_distance,checker,sampler,metrics);
[success,path4]=solver4.solve;
if ~success
    fprintf('BiRRT Connect failed\n');
    return
end
path4=path4.resample(connection_max_length,metrics);


path_optimizer=PathLocalOptimizer(path4,opt_type,checker,metrics);
path_optimizer.solve;
fprintf('BiRRT Connect: cost=%f\n',path4.cost);


joints=path4.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','--c','LineWidth',1)

%% 5° Path
solver5 = BirrtConnect(start_conf,goal_conf,max_distance,checker,sampler,metrics);
[success,path5]=solver5.solve;
if ~success
    fprintf('BiRRT Connect failed\n');
    return
end
path5=path5.resample(connection_max_length,metrics);


path_optimizer=PathLocalOptimizer(path5,opt_type,checker,metrics);
path_optimizer.solve;
fprintf('BiRRT Connect: cost=%f\n',path5.cost);


joints=path5.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','--m','LineWidth',1)

%% 
current_path = path1;
other_paths = [path2 path3 path4 path5];

succ_node = 1;
idx_replan=round(length(path1.connections)*0.5);
node=current_path.connections(:,idx_replan).getChild;

[new_path,path_cost,success,con2path] = PathSwitch(current_path,other_paths,node,lb,ub,max_distance,checker,metrics,opt_type,succ_node);
if(success == 1)
    joints=new_path.getWaypoints;
    plot3(joints(1,:)',joints(2,:)',joints(3,:)','-y','LineWidth',2)
else
    warning('replanning not possible!');
end

