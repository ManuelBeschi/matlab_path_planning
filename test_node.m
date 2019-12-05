clear all;close all;clc;

% inizializza ROS (se non lo è già')
try
    rosinit
end



robot=importrobot('sharework_cell.urdf');
home_conf=homeConfiguration(robot);

lb=-pi*ones(7,1); % leggi da urdf
ub=pi*ones(7,1);
lb(7,1)=-0.6;
ub(7,1)=0.6;

checker=CollisionChecker;
checker.init('cembre',true);
names=checker.getNames;


home = [-1.0000   -3.1414    2.5708   -1.0000    1.5708   -0.0001   -0.000]';
down = [1.5707   -1.5571    1.5708   -1.5708    1.5707   -0.0001    0.0001]';
p1   = [-0.5000   -3.1414    2.5708   -1.0000    1.5708   -0.0001   -0.000]';

start_node=node(home);
goal_node=node(down);

max_distance=0.5;

start_tree=tree(start_node,1,max_distance,checker);
goal_tree=tree(goal_node,0,max_distance,checker);

success=false;
for idx=1:1000
    q=lb+(ub-lb)*rand;
    [add_to_start,new_t1_node]=start_tree.extend(q);
    if (add_to_start)
        [add_to_goal,new_t2_node]=goal_tree.extendToNode(new_t1_node);
    else
        [add_to_goal,new_t2_node]=goal_tree.extend(q);
    end
    if (add_to_goal && add_to_start && isequal(new_t1_node,new_t2_node))
        success=true;
        break;
    end
    if mod(idx,100)==0
        fprintf('idx=%d\n',idx);
    end
end

if (~success)
    fprintf('failed');
end
path=[start_tree.getConnectionToNode(new_t1_node) goal_tree.getConnectionToNode(new_t1_node)];
%%
joints=path(1).getParent.q;
for in=1:length(path)
    joints(:,in+1)=path(in).getChild.q;
end
figure(1)
plot(joints')


xyz=zeros(length(path)+1,3);
for ij=1:size(joints,2)
    for iax=1:length(home_conf)
        conf(iax).JointName=names{iax};
        conf(iax).JointPosition=joints(iax,ij);
    end
    T=robot.getTransform(conf,'tool');
    xyz(ij,:)=T(1:3,4)';
end
figure(2)
plot(xyz)
