clear all;close all;clc;

obstacle='sphere';
% obstacle='cube';
% obstacle='snowman';

opttype='slip';
opttype='slipParent';
opttype='slipChild';
opttype='warp';
opttype='spiral';
opttype='full';


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


lb=-pi*ones(3,1);
ub=pi*ones(3,1);

start_conf = [1.1 0 0]';
pos1=[1.2 0 2]';
pos2=[0 1.2 3]';
goal_conf = [0 1.1 0]';
n1=Node(start_conf);
n2=Node(pos1);
n3=Node(pos2);
n4=Node(goal_conf);
c12=Connection(n1,n2);
c23=Connection(n2,n3);
c34=Connection(n3,n4);

path=Path([c12 c23 c34]);
path=path.resample(0.4);


hold on
joints=path.getWaypoints;
plot3(joints(1,:)',joints(2,:)',joints(3,:)','LineWidth',2)
plot3(start_conf(1),start_conf(2),start_conf(3),'ob','MarkerFaceColor','b','MarkerSize',5)
plot3(goal_conf(1),goal_conf(2),goal_conf(3),'or','MarkerFaceColor','r','MarkerSize',5)

view(135,0)
cost_iter=[];

lines=[];
for idx=1:50
    if strcmp(opttype,'slip')
        cost_iter=[cost_iter;path.slipParent(checker)];
        cost_iter=[cost_iter;path.slipChild(checker)];
    elseif strcmp(opttype,'slipParent')
        cost_iter=[cost_iter;path.slipParent(checker)];
    elseif strcmp(opttype,'slipChild')
        cost_iter=[cost_iter;path.slipChild(checker)];
    elseif strcmp(opttype,'warp')
        cost_iter=[cost_iter;path.warp(checker)];
    elseif strcmp(opttype,'spiral')
        cost_iter=[cost_iter;path.spiral(checker)];
    elseif strcmp(opttype,'full')
        cost_iter=[cost_iter;path.slipParent(checker)];
        cost_iter=[cost_iter;path.slipChild(checker)];
        cost_iter=[cost_iter;path.warp(checker)];
        cost_iter=[cost_iter;path.spiral(checker)];
    end
    joints=path.getWaypoints;
    for il=1:length(lines)
        lines(il).Color=lines(il).Color+[0.05 0.05 0.05];
        lines(il).LineWidth=lines(il).LineWidth*0.9;
        if lines(il).Color(1)>0.9
            delete(lines(il))
        end
    end
    subplot(121)
    l=plot3(joints(1,:)',joints(2,:)',joints(3,:)','k','LineWidth',2);
    lines=[lines l];
    lines=lines(isvalid(lines));
    axis equal
    xlabel('q1');
    ylabel('q2');
    zlabel('q3');
    
    subplot(122)
    
    subplot(122)
    stairs(cost_iter)
    xlabel('iteration')
    ylabel('cost')
    drawnow
end