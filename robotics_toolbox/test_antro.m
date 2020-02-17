clear all;close all;clc;


% checker -> collision checker
checker=Anthropomorphic3d;
checker.init;
metrics=Metrics;

% lower and upper bound
lb=-pi*ones(3,1);
ub=pi*ones(3,1);
% velocity bound
max_vel=2*ones(3,1);

start_conf = [0 0.2 pi/2+0.4]'; % punto di partenza nelle configurazioni
goal_conf = [-pi/2 0.2 pi/2+0.4]'; %  punto di partenza nelle configurazioni


sampler = InformedSampler(start_conf,goal_conf,lb,ub);


passo_crescita=1; % passo di crescita albero (massimma distanza fra nodi)
solver = BirrtConnect(start_conf,goal_conf,passo_crescita,checker,sampler,metrics);

[success,path]=solver.solve;
if ~success
    fprintf('BiRRT Connect failed\n');
    return
end

fprintf('BiRRT Connect: cost=%f\n',path.cost);

% il robotic toolbox è molto lento, ho ridotto l'ottimizzazione locale a 2 iterazioni 
path_optimizer=PathLocalOptimizer(path,'full',checker,metrics);
path_optimizer.solve(2);
fprintf('BiRRT-Connect Path local opt cost=%f\n',path.cost);

[~,~,total_duration]=bound_velocity_time_parametrization(path,max_vel);

total_duration
%% only visualization

figure('Position',[10 10 1200 600])
drawnow
st=1e-2;
time=(0:st:total_duration)';
for idx=1:length(time)
    [pos,vel,~]=bound_velocity_time_parametrization(path,max_vel,time(idx));
    checker.plot3d(pos)
    pause(st)
end

