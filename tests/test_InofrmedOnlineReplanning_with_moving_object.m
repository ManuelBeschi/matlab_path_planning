clearvars;close all;clc;

% obstacle='sphere';
% obstacle='cube';
obstacle='snowman';
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
    error('invalid ostacle')
end
checker.init;

figure('Position',[10 10 1200 600])
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
    start_conf = [1.1 0 0]';
    goal_conf = [-0.6 1.1 0]';
end
i = 1;

hold on
plot3(start_conf(1),start_conf(2),start_conf(3),'ob','MarkerFaceColor','b','MarkerSize',5)
plot3(goal_conf(1),goal_conf(2),goal_conf(3),'or','MarkerFaceColor','r','MarkerSize',5)

connection_max_length=0.5;
max_distance = 0.5;

% sampler = InformedSampler(start_conf,goal_conf,lb,ub);
% solver1 = BirrtConnect(start_conf,goal_conf,max_distance,checker,sampler,metrics);
%
% [success,path]=solver1.solve;
% if ~success
%     fprintf('BiRRT Connect failed\n');
%     return
% end
%
% fprintf('BiRRT Connect: cost=%f\n',path.cost);
%solver1.start_tree.plot
%
% path_optimizer=PathLocalOptimizer(path,opt_type,checker,metrics);
% path_optimizer.solve;
%
% joints=path.getWaypoints;
% plot3(joints(1,:)',joints(2,:)',joints(3,:)','m','LineWidth',2)
% fprintf('BiRRT-Extend Path local opt cost=%f\n',path.cost);
% xlabel('x')
% ylabel('y')
% zlabel('z')
%% 1° Path
sampler = InformedSampler(start_conf(:,i),goal_conf(:,i),lb,ub);
solver1 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
[success,path1]=solver1.solve; %#ok<*ASGLU>
path1=path1.resample(connection_max_length,metrics);

path_optimizer=PathLocalOptimizer(path1,opt_type,checker,metrics);
path_optimizer.solve;

%         path1.connections(end).setCost(30); % NB: così sei sicuro che lo switch avverrà
%         path1.connections(end).setCost(inf);

joints=path1.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','Color','blue','LineWidth',1)

%% 2° Path
solver2 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
[success,path2]=solver2.solve;
path2=path2.resample(connection_max_length,metrics);

path_optimizer=PathLocalOptimizer(path2,opt_type,checker,metrics);
path_optimizer.solve;

joints=path2.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','Color','red','LineWidth',1)

%% 3° Path
solver3 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
[success,path3]=solver3.solve;
path3=path3.resample(connection_max_length,metrics);

path_optimizer=PathLocalOptimizer(path3,opt_type,checker,metrics);
path_optimizer.solve;

joints=path3.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','Color','green','LineWidth',1)

%% 4° Path
solver4 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
[success,path4]=solver4.solve;
path4=path4.resample(connection_max_length,metrics);

path_optimizer=PathLocalOptimizer(path4,opt_type,checker,metrics);
path_optimizer.solve;

joints=path4.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','Color','black','LineWidth',1)

%% 5° Path
solver5 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
[success,path5]=solver5.solve;
path5=path5.resample(connection_max_length,metrics);

path_optimizer=PathLocalOptimizer(path5,opt_type,checker,metrics);
path_optimizer.solve;

joints=path5.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','Color','white','LineWidth',1)

%%
max_vel=2*ones(3,1);
st=1e-2;
[pos,vel,total_duration]=bound_velocity_time_parametrization(path1,max_vel,0);
t=(0:st:total_duration)';

obstacle_changed=false;
already_replanned = false;

current_path = path1;
other_paths = [path2,path3,path4,path5];
idx = 1;
while(idx<=length(t))
    if (~obstacle_changed && t(idx)>.3)
        checker=Snowman3dWithPoleCollisionChecker;
        checker.init;
        checker.plot
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
    end
    
    h=plot3(pos(1,end),pos(2,end),pos(3,end),'or','MarkerFaceColor','r');

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
        path_optimizer.solve(100);
        joints=replanned_path.getWaypoints;
        plot3(joints(1,:)',joints(2,:)',joints(3,:)','Color','yellow','LineWidth',1)
        
        already_replanned = true;
        idx = 1;
        pos = [];
        vel = [];
    end
    
    if(already_replanned)
        [pos(:,idx),vel(:,idx),total_duration]=bound_velocity_time_parametrization(replanned_path,max_vel,t(idx));
        t=(0:st:total_duration)';
    end
    
    drawnow
    pause(st)
    
%     if ~path1.valid(checker)
%         return
%     end
    
    delete(h)
    idx = idx+1;
end
