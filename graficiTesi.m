clearvars; close all; clc; warning off;

connection_max_length=0.5;
%obstacle='sphere';
obstacle='cube';
%obstacle='snowman';
%obstacle='torus';
opt_type='full';

verbose = 0;

if strcmp(obstacle,'snowman')
    checker=Snowman3dCollisionChecker;
elseif strcmp(obstacle,'cube')
    checker=Cube3dCollisionChecker;
elseif strcmp(obstacle,'sphere')
    checker=Sphere3dCollisionChecker;
elseif strcmp(obstacle,'torus')
    checker=Torus3dCollisionChecker;
else
    error('invalid obstacle')
end
checker.init;

if(verbose < 2)
    figure('Position',[100 100 1200 600])
    checker.plot
end
metrics=Metrics;
if(verbose < 2)
 view(135,0)
end

lb=-pi*ones(3,1);
ub=pi*ones(3,1);

if strcmp(obstacle,'torus')
start_conf = [0 1.1 0]';
goal_conf = [0.6 -0.7 0]';
else
start_conf = [0.0 -2.5 0]';
goal_conf =  [0.0 2.5 0]';
end

if(verbose < 2)
hold on

axis equal
xlabel('q1');
ylabel('q2');
zlabel('q3');
end

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
path_optimizer.solve(1);

%path1.connections(end).setCost(30); % NB: così sei sicuro che lo switch avverrà
path1.connections(end).setCost(inf);

fprintf('BiRRT Connect: cost=%f\n',path1.cost);

joints=path1.getWaypoints;
if(verbose < 2)
    plot3(joints(1,:)',joints(2,:)',joints(3,:)','-b','LineWidth',2)
    plot3(joints(1,2:end-1)',joints(2,2:end-1)',joints(3,2:end-1)','ob','LineWidth',3,'MarkerFaceColor','b','MarkerSize',3)
    plot3(start_conf(1),start_conf(2),start_conf(3),'og','MarkerFaceColor','y','MarkerSize',5)
plot3(goal_conf(1),goal_conf(2),goal_conf(3),'or','MarkerFaceColor','r','MarkerSize',5)
end
% %% 2° Path
% solver2 = BirrtConnect(start_conf,goal_conf,max_distance,checker,sampler,metrics);
% [success,path2]=solver2.solve;
% if ~success
%     fprintf('BiRRT Connect failed\n');
%     return
% end
% path2=path2.resample(connection_max_length,metrics);
% 
% 
% path_optimizer=PathLocalOptimizer(path2,opt_type,checker,metrics);
% path_optimizer.solve;
% fprintf('BiRRT Connect: cost=%f\n',path2.cost);
% 
% 
% joints=path2.getWaypoints;
% if(verbose < 2)
%     plot3(joints(1,:)',joints(2,:)',joints(3,:)','--r','LineWidth',1)
% end
% %% 3° Path
% solver3 = BirrtConnect(start_conf,goal_conf,max_distance,checker,sampler,metrics);
% [success,path3]=solver3.solve;
% if ~success
%     fprintf('BiRRT Connect failed\n');
%     return
% end
% path3=path3.resample(connection_max_length,metrics);
% 
% 
% path_optimizer=PathLocalOptimizer(path3,opt_type,checker,metrics);
% path_optimizer.solve;
% fprintf('BiRRT Connect: cost=%f\n',path3.cost);
% 
% 
% joints=path3.getWaypoints;
% if(verbose < 2)
%     plot3(joints(1,:)',joints(2,:)',joints(3,:)','--g','LineWidth',1)
% end
% %% 
% current_path = path1;
% other_paths = [path2 path3];
% 
% succ_node = 1;
% informed = 2;
% idx_replan=round(length(path1.connections)*0.5);
% child=current_path.connections(:,idx_replan).getChild.q;
% parent=current_path.connections(:,idx_replan).getParent.q;
% q = (child+parent)/2;
% %current_path.connections(:,idx_replan).setCost(inf);
% 
% if(verbose < 2)
%     path1_nodes = path1.getWaypoints;
%     path2_nodes = path2.getWaypoints;
%     path3_nodes = path3.getWaypoints;
%     plot3(q(1,:)',q(2,:)',q(3,:)','*c','LineWidth',2)
%     plot3(path1_nodes(1,:)',path1_nodes(2,:)',path1_nodes(3,:)','*b','LineWidth',0.5)
%     plot3(path2_nodes(1,:)',path2_nodes(2,:)',path2_nodes(3,:)','*r','LineWidth',0.5)
%     plot3(path3_nodes(1,:)',path3_nodes(2,:)',path3_nodes(3,:)','*g','LineWidth',0.5)
% end
% tic
% [replanned_path,replanned_path_cost,success,replanned_path_vector,number_replanning] = InformedOnlineReplanning(current_path,other_paths,q,lb,ub,max_distance,checker,metrics,opt_type,succ_node,informed,verbose);
% toc
% if(isa(replanned_path,'Path'))
%     joints=replanned_path.getWaypoints;
%     plot3(joints(1,:)',joints(2,:)',joints(3,:)','--y','LineWidth',1)
%     plot3(joints(1,:)',joints(2,:)',joints(3,:)','oy','LineWidth',1)
%     replanned_path.verboseDebug(false);
%     path_optimizer=PathLocalOptimizer(replanned_path,opt_type,checker,metrics);
%     path_optimizer.solve;
%     joints=replanned_path.getWaypoints;
%     plot3(joints(1,:)',joints(2,:)',joints(3,:)','-y','LineWidth',1.5)
% else
%     warning('replanning not possible!');
% end

%Legenda:
%Blu: path iniziale
%Verde, Rosso: path alternativi
%Giallo: tratteggiato->path trovato, continuo spesso->path trovato smussato
%Ciano: penultimo path trovato