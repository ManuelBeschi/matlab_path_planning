clear all;close all;clc;
    
% obstacle='sphere';
obstacle='cube';
% obstacle='snowman';

% opt_type='slip';
% opt_type='slipParent';
% opt_type='slipChild';
% opt_type='warp';
% opt_type='spiral'; % DEPRECATED
opt_type='full';

connection_max_length=0.2;
% connection_max_length=2;

if strcmp(obstacle,'snowman')
    checker=Snowman3dCollisionChecker;
elseif strcmp(obstacle,'cube')
    checker=Cube3dCollisionChecker;
elseif strcmp(obstacle,'sphere')
    checker=Sphere3dCollisionChecker;
else
    error('invalid ostable')
end
checker.init;

%%
figure('Position',[10 10 1200 600])
subplot(121)
checker.plot

metrics=Metrics;

lb=-pi*ones(3,1);
ub=pi*ones(3,1);

start_conf = [1.1 0 0]';
pos1=[1.2 0 2]';
pos2=[0 1.2 3]';
goal_conf = [-0.6 1.1 0]';
n1=Node(start_conf);
n2=Node(pos1);
n3=Node(pos2);
n4=Node(goal_conf);
c12=Connection(n1,n2,metrics.cost(n1,n2));
c23=Connection(n2,n3,metrics.cost(n2,n3));
c34=Connection(n3,n4,metrics.cost(n3,n4));

path=Path([c12 c23 c34]);
path=path.resample(connection_max_length,metrics);


hold on
joints=path.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','LineWidth',2)
plot3(start_conf(1),start_conf(2),start_conf(3),'ob','MarkerFaceColor','b','MarkerSize',5)
plot3(goal_conf(1),goal_conf(2),goal_conf(3),'or','MarkerFaceColor','r','MarkerSize',5)

view(135,0)
% path.verboseDebug(1)
path_optimizer=PathLocalOptimizer(path,opt_type,checker,metrics);
cost=[];
lines=[];
for idx=1:100
    solved=path_optimizer.step;
    joints=path.getWaypoints;
    for il=1:length(lines)
        lines(il).Color=1-(1-lines(il).Color)*0.92;%+[0.05 0.05 0.05];
        lines(il).LineWidth=0.3;
        if lines(il).Color(1)>0.9
            delete(lines(il))
        end
    end
    cost(idx,1)=path.cost;
    
    subplot(121)
    l=plot3(joints(1,:)',joints(2,:)',joints(3,:)','k','LineWidth',2);
    lines=[lines l];
    lines=lines(isvalid(lines));
    axis equal
    xlabel('q1');
    ylabel('q2');
    zlabel('q3');
    
    subplot(122)
    plot(cost)
    xlabel('iteration')
    ylabel('cost');
     subplot(121)
    drawnow
    if solved
        break;
    end
end