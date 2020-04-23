clear all;close all;

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

hold on
plot3(start_conf(1),start_conf(2),start_conf(3),'ob','MarkerFaceColor','b','MarkerSize',5)
plot3(goal_conf(1),goal_conf(2),goal_conf(3),'or','MarkerFaceColor','r','MarkerSize',5)

max_distance=0.5;

sampler = InformedSampler(start_conf,goal_conf,lb,ub);
solver1 = BirrtConnect(start_conf,goal_conf,max_distance,checker,sampler,metrics);

[success,path]=solver1.solve;
if ~success
    fprintf('BiRRT Connect failed\n');
    return
end

fprintf('BiRRT Connect: cost=%f\n',path.cost);
solver1.start_tree.plot

path_optimizer=PathLocalOptimizer(path,opt_type,checker,metrics);
path_optimizer.solve;

joints=path.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','m','LineWidth',2)
fprintf('BiRRT-Extend Path local opt cost=%f\n',path.cost);
xlabel('x')
ylabel('y')
zlabel('z')

max_vel=2*ones(3,1);
st=1e-2;
[pos,vel,total_duration]=bound_velocity_time_parametrization(path,max_vel,0);
t=(0:st:total_duration)';

obstacle_changed=false;
for idx=1:length(t)
    if (~obstacle_changed &&t(idx)>.3)
        checker=Snowman3dWithPoleCollisionChecker;
        checker.init;
        checker.plot
        obstacle_changed=true;
    end
    [pos(:,idx),vel(:,idx)]=bound_velocity_time_parametrization(path,max_vel,t(idx));
    h=plot3(pos(1,idx),pos(2,idx),pos(3,idx),'or','MarkerFaceColor','r');
    drawnow
    pause(st)
    
    if ~path.valid(checker)
        return
    end
    
    delete(h)
end
