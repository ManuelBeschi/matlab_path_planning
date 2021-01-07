clear all;close all;clc;




% lower and upper bound
lb=-pi*ones(3,1);
ub=pi*ones(3,1);
% velocity bound
max_vel=2*ones(3,1);


offset=[0;0;0]; % posizione della base del robot in questo test
    
% checker -> collision checker
checker=Anthropomorphic3d;
checker.init(lb,ub,offset);
metrics=Metrics;


A = Node([0 0.2 pi/2+0.4]'); % punto di partenza nelle configurazioni
B =  Node([-pi/2 0.2 pi/2+0.4]'); %  punto di partenza nelle configurazioni
C =  Node([-pi/2 0.6 pi/2+0.4]'); %  punto di partenza nelle configurazioni

%%
sampler_AB = InformedSampler(A,B,lb,ub);

passo_crescita=1; % passo di crescita albero (massimma distanza fra nodi)
solver_AB = BirrtConnect(A,B,passo_crescita,checker,sampler_AB,metrics);

[success,path_AB]=solver_AB.solve;
if ~success
    fprintf('BiRRT Connect failed\n');
    return
end

fprintf('BiRRT Connect: cost=%f\n',path_AB.cost);

% il robotic toolbox è molto lento, ho ridotto l'ottimizzazione locale a 2 iterazioni 
path_optimizer_AB=PathLocalOptimizer(path_AB,'full',checker,metrics);
path_optimizer_AB.solve(2);
fprintf('BiRRT-Connect Path local opt cost=%f\n',path_AB.cost);


%%
sampler_BC = InformedSampler(B,C,lb,ub);

passo_crescita=1; % passo di crescita albero (massimma distanza fra nodi)
solver_BC = BirrtConnect(B,C,passo_crescita,checker,sampler_BC,metrics);

[success,path_BC]=solver_BC.solve;
if ~success
    fprintf('BiRRT Connect failed\n');
    return
end

fprintf('BiRRT Connect: cost=%f\n',path_BC.cost);

% il robotic toolbox è molto lento, ho ridotto l'ottimizzazione locale a 2 iterazioni 
path_optimizer_BC=PathLocalOptimizer(path_BC,'full',checker,metrics);
path_optimizer_BC.solve(2);
fprintf('BiRRT-Connect Path local opt cost=%f\n',path_BC.cost);



%%
path_AC=Path([path_AB.connections path_BC.connections]);

[~,~,total_duration]=bound_velocity_time_parametrization(path_AC,max_vel);


total_duration
%% only visualization

figure('Position',[10 10 1200 600])
drawnow
st=1e-2;
time=(0:st:total_duration)';
for idx=1:length(time)
    [pos,vel,~]=bound_velocity_time_parametrization(path_AC,max_vel,time(idx));
    checker.plot3d(pos)
    pause(st)
end

