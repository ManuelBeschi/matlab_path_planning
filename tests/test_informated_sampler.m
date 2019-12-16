clear all;close all;

start_conf = [1.1 0 0]';
goal_conf = [-1 1.1 1]';


lb=-pi*ones(3,1);
ub=pi*ones(3,1);

cost=2.8;

if 0
    sampler = InformedSampler(start_conf,goal_conf,lb,ub,cost);
else
    sampler = LocalInformedSampler(start_conf,goal_conf,lb,ub,cost);
    sampler.setBall([0.5 0.5 0.5]',.1);
end

sampler.plotEllipsoid
hold on

for idx=1:100
    q=sampler.sample;
    plot3(q(1),q(2),q(3),'ok')
end
axis equal