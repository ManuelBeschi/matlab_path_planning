clear all;close all;

% obstacle='sphere';
% obstacle='cube';
obstacle='snowman';
obstacle='torus';
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

figure('Position',[10 10 1200 600])
% subplot(121)
checker.plot

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
solver = BirrtExtend(start_conf,goal_conf,max_distance,checker,sampler);
solver1 = BirrtConnect(start_conf,goal_conf,max_distance,checker,sampler);

[success,path]=solver1.solve;
if ~success
    fprintf('BiRRT Connect failed\n');
    return
end

fprintf('BiRRT Connect: cost=%f\n',path.cost);
solver1.start_tree.plot

joints=path.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','b','LineWidth',1)

lines=[];
cost_iter=[];
for idx=1:10
    if strcmp(opt_type,'slip')
        cost_iter=[cost_iter;path.slipParent(checker)];
        cost_iter=[cost_iter;path.slipChild(checker)];
    elseif strcmp(opt_type,'slipParent')
        cost_iter=[cost_iter;path.slipParent(checker)];
    elseif strcmp(opt_type,'slipChild')
        cost_iter=[cost_iter;path.slipChild(checker)];
    elseif strcmp(opt_type,'warp')
        cost_iter=[cost_iter;path.warp(checker)];
    elseif strcmp(opt_type,'spiral')
        cost_iter=[cost_iter;path.spiral(checker)];
    elseif strcmp(opt_type,'full')
        cost_iter=[cost_iter;path.slipParent(checker)];
        cost_iter=[cost_iter;path.slipChild(checker)];
        cost_iter=[cost_iter;path.warp(checker)];
        cost_iter=[cost_iter;path.spiral(checker)];
    end
end
joints=path.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','b','LineWidth',2)

fprintf('BiRRT-Connect Path local opt cost=%f\n',path.cost);

 solver1.start_tree.addBranch(path.connections);
 solver1.start_tree.plot


% return
sampler.setCost(path.cost);
% sampler.plotEllipsoid
[success,path]=solver.solve;
if ~success
    fprintf('BiRRT extend failed\n');
    return
end
joints=path.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','m','LineWidth',1)

fprintf('BiRRT Extend cost=%f\n',path.cost);


cost_iter=[];
for idx=1:10
    if strcmp(opt_type,'slip')
        cost_iter=[cost_iter;path.slipParent(checker)];
        cost_iter=[cost_iter;path.slipChild(checker)];
    elseif strcmp(opt_type,'slipParent')
        cost_iter=[cost_iter;path.slipParent(checker)];
    elseif strcmp(opt_type,'slipChild')
        cost_iter=[cost_iter;path.slipChild(checker)];
    elseif strcmp(opt_type,'warp')
        cost_iter=[cost_iter;path.warp(checker)];
    elseif strcmp(opt_type,'spiral')
        cost_iter=[cost_iter;path.spiral(checker)];
    elseif strcmp(opt_type,'full')
        cost_iter=[cost_iter;path.slipParent(checker)];
        cost_iter=[cost_iter;path.slipChild(checker)];
        cost_iter=[cost_iter;path.warp(checker)];
        cost_iter=[cost_iter;path.spiral(checker)];
    end
end
joints=path.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','m','LineWidth',2)
fprintf('BiRRT-Extend Path local opt cost=%f\n',path.cost);


axis equal
xlabel('q1');
ylabel('q2');
zlabel('q3');
