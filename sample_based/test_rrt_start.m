clear all;close all;

% obstacle='sphere';
% obstacle='cube';
% obstacle='snowman';
% obstacle='torus';
obstacle='cubes';
opt_type='full';
informed=true;
local_opt=true;
if strcmp(obstacle,'snowman')
    checker=Snowman3dCollisionChecker;
elseif strcmp(obstacle,'cube')
    checker=Cube3dCollisionChecker;
elseif strcmp(obstacle,'cubes')
    checker=Cubes3dCollisionChecker;
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
axis equal
xlabel('q1');
ylabel('q2');
zlabel('q3');

view(135,0)

lb=-pi*ones(3,1);
ub=pi*ones(3,1);

if strcmp(obstacle,'torus')
    start_conf = [0 1.1 0]';
    goal_conf = [0.6 -0.7 0]';
elseif strcmp(obstacle,'snowman')
    start_conf = [0 0.6 1]';
    goal_conf = [0 -0.6 1]';
elseif strcmp(obstacle,'cubes')
%     flag=true;
%     while flag
%         start_conf = lb+(ub-lb).*rand(3,1);
%         if checker.check(start_conf)
%             break;
%         end
%     end
%     
%     while flag
%         goal_conf = lb+(ub-lb).*rand(3,1);
%         if checker.check(goal_conf)
%             break;
%         end
%     end
    start_conf=lb;
    goal_conf=ub;
else
    start_conf = [1.1 0 0]';
    goal_conf = [-0.6 1.1 0]';
end

if ~checker.check(start_conf)
    error('start conf is in collision');
end
if ~checker.check(goal_conf)
    error('goal_conf is in collision');
end

hold on
plot3(start_conf(1),start_conf(2),start_conf(3),'ob','MarkerFaceColor','b','MarkerSize',5)
plot3(goal_conf(1),goal_conf(2),goal_conf(3),'or','MarkerFaceColor','r','MarkerSize',5)
drawnow;
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

drawnow

if local_opt
    
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
end

joints=path.getWaypoints;
best_path_plot=plot3(joints(1,:)',joints(2,:)',joints(3,:)','b','LineWidth',4);

cost=path.cost;
goal_node=solver1.goal_tree.root;
if informed
    sampler.setCost(cost);
end
eli=sampler.plotEllipsoid;
drawnow
fprintf('BiRRT Connect Local Opt: cost=%f\n',path.cost);

for ii=1:200
    solver1.start_tree.rewire(sampler.sample,checker);
    if (solver1.start_tree.costToNode(goal_node)<(cost-1e-6))
        path=Path(solver1.start_tree.getConnectionToNode(goal_node));
        
        

        fprintf('RRT* iter cost=%f\n',path.cost);
        if local_opt
            
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
        fprintf('RRT* iter-local cost=%f\n',path.cost);
        end
        cost=path.cost;
        if informed
            sampler.setCost(cost);
        end
        delete(eli)
        eli=sampler.plotEllipsoid;
        delete(best_path_plot);
        joints=path.getWaypoints;
        best_path_plot=plot3(joints(1,:)',joints(2,:)',joints(3,:)','b','LineWidth',4);
        
        solver1.start_tree.plot
        drawnow
    end
    if mod(ii,100)==0
        
        solver1.start_tree.plot
        drawnow
    end
end

axis equal
xlabel('q1');
ylabel('q2');
zlabel('q3');


fprintf('RRT*: cost=%f\n',path.cost);
