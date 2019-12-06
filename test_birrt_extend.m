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
checker.init('cembre',0);
names=checker.getNames;


home = [-1.0000   -3.1414    2.5708   -1.0000    1.5708   -0.0001   -0.000]';
down = [1.5707   -1.5571    1.5708   -1.5708    1.5707   -0.0001    0.0001]';
p1   = [-0.5000   -3.1414    2.5708   -1.0000    1.5708   -0.0001   -0.000]';

max_distance=0.5;

sampler = informed_sampler(home,down,lb,ub);
solver=birrt_extend(home,down,max_distance,checker,sampler);

[success,opt_path]=solver.solve;

if (~success)
    fprintf('failed');
    return
end

%%
joints=opt_path.getWaypoints;

figure(1)
plot(joints')
xlabel('nodes');
ylabel('joint configuration');
%%
xyz=zeros(length(opt_path)+1,3);
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
xlabel('nodes');
ylabel('XYZ trajectory');
%%
