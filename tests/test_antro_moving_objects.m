clear all;close all;clc;

checker=Anthropomorphic3d;
checker.init;
metrics=Metrics;
connection_max_length=0.5;
max_distance = 0.5;
opt_type = 'full';

% lower and upper bound
lb=-pi*ones(3,1);
ub=pi*ones(3,1);
% velocity bound
max_vel=2*ones(3,1);

start_conf = [0 pi/9 -pi/6]'; 
goal_conf = [-pi/2 -pi/6 pi/6]'; 

hold on
figure(1)
plot3(start_conf(1),start_conf(2),start_conf(3),'ob','MarkerFaceColor','b','MarkerSize',5)
plot3(goal_conf(1),goal_conf(2),goal_conf(3),'or','MarkerFaceColor','r','MarkerSize',5)

i=1;
%% 1° Path
sampler = InformedSampler(start_conf(:,i),goal_conf(:,i),lb,ub);
solver1 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
[success,path1]=solver1.solve; %#ok<*ASGLU>
if ~success
    fprintf('BiRRT Connect failed\n');
    return
end

fprintf('BiRRT Connect: cost=%f\n',path1.cost);
path1=path1.resample(connection_max_length,metrics);

path_optimizer=PathLocalOptimizer(path1,opt_type,checker,metrics);
path_optimizer.solve;
fprintf('BiRRT-Connect Path local opt cost=%f\n',path1.cost);

joints=path1.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','-o','Color','blue','LineWidth',1)

%% 2° Path
solver2 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
[success,path2]=solver2.solve;
if ~success
    fprintf('BiRRT Connect failed\n');
    return
end

fprintf('BiRRT Connect: cost=%f\n',path2.cost);
path2=path2.resample(connection_max_length,metrics);

path_optimizer=PathLocalOptimizer(path2,opt_type,checker,metrics);
path_optimizer.solve;
fprintf('BiRRT-Connect Path local opt cost=%f\n',path2.cost);

joints=path2.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','-o','Color','red','LineWidth',1)

%% 3° Path
solver3 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
[success,path3]=solver3.solve;
if ~success
    fprintf('BiRRT Connect failed\n');
    return
end

fprintf('BiRRT Connect: cost=%f\n',path3.cost);
path3=path3.resample(connection_max_length,metrics);

path_optimizer=PathLocalOptimizer(path3,opt_type,checker,metrics);
path_optimizer.solve;
fprintf('BiRRT-Connect Path local opt cost=%f\n',path3.cost);

joints=path3.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','-o','Color','green','LineWidth',1)

%% 4° Path
solver4 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
[success,path4]=solver4.solve;
if ~success
    fprintf('BiRRT Connect failed\n');
    return
end

fprintf('BiRRT Connect: cost=%f\n',path4.cost);
path4=path4.resample(connection_max_length,metrics);

path_optimizer=PathLocalOptimizer(path4,opt_type,checker,metrics);
path_optimizer.solve;
fprintf('BiRRT-Connect Path local opt cost=%f\n',path4.cost);

joints=path4.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','-o','Color','black','LineWidth',1)

%% 5° Path
solver5 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
[success,path5]=solver5.solve;
if ~success
    fprintf('BiRRT Connect failed\n');
    return
end

fprintf('BiRRT Connect: cost=%f\n',path5.cost);
path5=path5.resample(connection_max_length,metrics);
fprintf('BiRRT-Connect Path local opt cost=%f\n',path5.cost);

path_optimizer=PathLocalOptimizer(path5,opt_type,checker,metrics);
path_optimizer.solve;

joints=path5.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','-o','Color','white','LineWidth',1)

%%
st = 1e-2;
[pos,vel,total_duration]=bound_velocity_time_parametrization(path1,max_vel,0);
t = 0:st:total_duration;

for k=1:1:length(t) %per simscape, per vedere il movimento del robot trasparente
    [q(:,k),v(:,k),tot_d]=bound_velocity_time_parametrization(path1,max_vel,t(k));
end

obstacle_changed=false;
already_replanned = false;

current_path = path1;
other_paths = [path2,path3,path4,path5];
idx = 1;
Q = [];
while(idx<=length(t))
    if (~obstacle_changed && t(idx)>.3)
        checker=Anthropomorphic3dWithMovingObstacle;
        checker.init;
        obstacle_changed=true;
        path1.valid(checker);
        path2.valid(checker);
        path3.valid(checker);
        path4.valid(checker);
        path5.valid(checker);
    end
    
    if(already_replanned == false)
        [pos(:,idx),vel(:,idx),total_duration]=bound_velocity_time_parametrization(path1,max_vel,t(idx));
        t=(0:st:total_duration)';
        Q = [Q,pos(:,idx)];
    end
    
    %figure(1)
    %h=plot3(pos(1,end),pos(2,end),pos(3,end),'or','MarkerFaceColor','r');

    if(obstacle_changed  && already_replanned == false && path1.cost == Inf)
        succ_node = 1;
        informed = 2; 
        verbose = 0;
        [replanned_path,replanned_path_cost,success,replanned_path_vector,number_replanning] = InformedOnlineReplanning(current_path,other_paths,pos(:,end),lb,ub,max_distance,checker,metrics,opt_type,succ_node,informed,verbose);
        toc
        if(replanned_path.cost == Inf)
            solver_replanned = BirrtConnect(pos(:,idx),goal_conf(:,i),max_distance,checker,sampler,metrics);
            [success,replanned_path]=solver_replanned.solve;
            replanned_path=replanned_path.resample(connection_max_length,metrics);
            disp('Replanned to GOAL because all the paths are invalid')
        end
        path_optimizer=PathLocalOptimizer(replanned_path,opt_type,checker,metrics);
        path_optimizer.solve;
        joints=replanned_path.getWaypoints;
        plot3(joints(1,:)',joints(2,:)',joints(3,:)','-o','Color','yellow','LineWidth',2)
        
        already_replanned = true;
        idx = 1;
        pos = [];
        vel = [];
    end
    
    if(already_replanned)
        [pos(:,idx),vel(:,idx),total_duration]=bound_velocity_time_parametrization(replanned_path,max_vel,t(idx));
        t=(0:st:total_duration)';
        Q = [Q,pos(:,idx)];
    end
    
    drawnow
    figure(2)
    try
    checker.plot3d(pos(:,end))
    catch
        a=0;
    end
    view(90,30)
    pause(st)
    %delete(h)
    idx = idx+1;
end

%% Simscape Multibody
param;
T = 5;
dt = T/(numel(Q(1,:))-1);
t = 0:dt:T;

Q1 = Q(1,:);
Q2 = Q(2,:);
Q3 = Q(3,:);

Q1 = [t',Q1'];
Q2 = [t',Q2'];
Q3 = [t',Q3'];

dt = T/(numel(q(1,:))-1);
t = 0:dt:T;

q1 = q(1,:);
q2 = q(2,:);
q3 = q(3,:);

q1 = [t',q1'];
q2 = [t',q2'];
q3 = [t',q3'];

sim('AT_robotics_replanning.mdl');