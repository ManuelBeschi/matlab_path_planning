clearvars; close all; clc; warning off;

connection_max_length=0.5;
obstacle='snowman';
opt_type='full';
metrics=Metrics;
max_distance=0.5;

verbose = 0;

checker=Snowman3dCollisionChecker;
checker.init;

start1 = [0,-1.1,0]';
start2 = [0,-1.1,-1]';
start3 = [-0.8,-0.8,0]';
start4 = [-0.5,-0.5,1.2]';
start5 = [1.3,0,-0.5]';
start6 = [1,-0.8,-0.6]';
start7 = [0,0,2]';
start8 =  [0,-1.5,0.5]';
start9 = [-1,-0.5,-0.5]';
start10 = [1,-1,-0.5]';
start11 = [1,0,-0.5]';
start12 = [1,0.5,0]';
start13 = [0.5,-1,1.1]';
start14 = [1,-0.7,0]';
start15 = [-1,0.2,-0.7]';
start16 = [0,-0.7,-1]';
start17 = [-1.2,0,-0.5]';
start18 =  [0,-1.5,0.7]';
start19 = [-1,-0.6,-1]';
start20 = [-0.5,0,-1]';

goal1 = [0,1.1,0]';
goal2 = [0,1.1,1]';
goal3 = [0.8,0.8,0]';
goal4 = [0.5,0.5,-1.2]';
goal5 = [-1.3,0,0.5]';
goal6 = [-1,0.8,0.6]';
goal7 = [0,0,-1.3]';
goal8 = [0,1.5,0.5]';
goal9 = [1,1,1]';
goal10 = [-1,1,0.5]';
goal11 = [-1,-1,-0.5]';
goal12 = [-1,0.5,0]';
goal13 = [0,0,-1.3]';
goal14 = [-0.7,0.5,1]';
goal15 = [1,0,0.5]';
goal16 = [0.7,0.8,0.6]';
goal17 = [0.7,0,1.5]';
goal18 = [0,1.5,0.7]';
goal19 = [1,0.8,0.5]';
goal20 = [1,1,1]';

start_conf = [start1,start2,start3,start4,start5,start6,start7,start8,start9,start10,start11,start12,start13,start14,start15,start16,start17,start18,start19,start20];
goal_conf = [goal1,goal2,goal3,goal4,goal5,goal6,goal7,goal8,goal9,goal10,goal11,goal12,goal13,goal14,goal15,goal16,goal17,goal18,goal19,goal20];

for i=1:1:length(start_conf(1,:))

    
    %% Figure
    
    %figure('Position ',[100 100 1200 600])
    fig = figure(i);
    checker.plot
    
    view(135,0)
    
    lb=-pi*ones(3,1);
    ub=pi*ones(3,1);
    
    hold on
    plot3(start_conf(1,i),start_conf(2,i),start_conf(3,i),'sy','MarkerFaceColor','b','MarkerSize',5)
    plot3(goal_conf(1,i),goal_conf(2,i),goal_conf(3,i),'oy','MarkerFaceColor','r','MarkerSize',5)
    
    axis equal
    xlabel('q1');
    ylabel('q2');
    zlabel('q3');
    
    %% 1° Path
    sampler = InformedSampler(start_conf(:,i),goal_conf(:,i),lb,ub);
    solver1 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
    [success,path1]=solver1.solve; %#ok<*ASGLU>
    path1=path1.resample(connection_max_length,metrics);
    
    path_optimizer=PathLocalOptimizer(path1,opt_type,checker,metrics);
    path_optimizer.solve;
    
    %         path1.connections(end).setCost(30); % NB: così sei sicuro che lo switch avverrà
    %         path1.connections(end).setCost(inf);
    
    joints=path1.getWaypoints;
    plot3(joints(1,:)',joints(2,:)',joints(3,:)','Color','blue','LineWidth',1)
    
    %% 2° Path
    solver2 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
    [success,path2]=solver2.solve;
    path2=path2.resample(connection_max_length,metrics);
    
    path_optimizer=PathLocalOptimizer(path2,opt_type,checker,metrics);
    path_optimizer.solve;
    
    joints=path2.getWaypoints;
    plot3(joints(1,:)',joints(2,:)',joints(3,:)','Color','red','LineWidth',1)
    
    %% 3° Path
    solver3 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
    [success,path3]=solver3.solve;
    path3=path3.resample(connection_max_length,metrics);
    
    path_optimizer=PathLocalOptimizer(path3,opt_type,checker,metrics);
    path_optimizer.solve;
    
    joints=path3.getWaypoints;
    plot3(joints(1,:)',joints(2,:)',joints(3,:)','Color','green','LineWidth',1)
    
    %% 4° Path
    solver4 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
    [success,path4]=solver4.solve;
    path4=path4.resample(connection_max_length,metrics);
    
    path_optimizer=PathLocalOptimizer(path4,opt_type,checker,metrics);
    path_optimizer.solve;
    
    joints=path4.getWaypoints;
    plot3(joints(1,:)',joints(2,:)',joints(3,:)','Color','black','LineWidth',1)
    
    %% 5° Path
    solver5 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
    [success,path5]=solver5.solve;
    path5=path5.resample(connection_max_length,metrics);
    
    path_optimizer=PathLocalOptimizer(path5,opt_type,checker,metrics);
    path_optimizer.solve;
    
    joints=path5.getWaypoints;
    plot3(joints(1,:)',joints(2,:)',joints(3,:)','Color','white','LineWidth',1)
    
    %% 6° Path
    solver6 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
    [success,path6]=solver6.solve; %#ok<*ASGLU>
    path6=path6.resample(connection_max_length,metrics);
    
    path_optimizer=PathLocalOptimizer(path6,opt_type,checker,metrics);
    path_optimizer.solve;
    
    %         path1.connections(end).setCost(30); % NB: così sei sicuro che lo switch avverrà
    %         path1.connections(end).setCost(inf);
    
    joints=path6.getWaypoints;
    plot3(joints(1,:)',joints(2,:)',joints(3,:)','Color','#0072BD','LineWidth',1)
    
    %% 7° Path
    solver7 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
    [success,path7]=solver7.solve;
    path7=path7.resample(connection_max_length,metrics);
    
    path_optimizer=PathLocalOptimizer(path7,opt_type,checker,metrics);
    path_optimizer.solve;
    
    joints=path7.getWaypoints;
    plot3(joints(1,:)',joints(2,:)',joints(3,:)','Color','#EDB120','LineWidth',1)
    
    %% 8° Path
    solver8 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
    [success,path8]=solver8.solve;
    path8=path8.resample(connection_max_length,metrics);
    
    path_optimizer=PathLocalOptimizer(path8,opt_type,checker,metrics);
    path_optimizer.solve;
    
    joints=path8.getWaypoints;
    plot3(joints(1,:)',joints(2,:)',joints(3,:)','Color','#7E2F8E','LineWidth',1)
    
    %% 9° Path
    solver9 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
    [success,path9]=solver9.solve;
    path9=path9.resample(connection_max_length,metrics);
    
    path_optimizer=PathLocalOptimizer(path9,opt_type,checker,metrics);
    path_optimizer.solve;
    
    joints=path9.getWaypoints;
    plot3(joints(1,:)',joints(2,:)',joints(3,:)','Color','#77AC30','LineWidth',1)
    
    %% 10° Path
    solver10 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
    [success,path10]=solver10.solve;
    path10=path10.resample(connection_max_length,metrics);
    
    path_optimizer=PathLocalOptimizer(path10,opt_type,checker,metrics);
    path_optimizer.solve;
    
    joints=path10.getWaypoints;
    plot3(joints(1,:)',joints(2,:)',joints(3,:)','Color','#A2142F','LineWidth',1)
    
    %% Saving
    path_vector = [path1,path2,path3,path4,path5,path6,path7,path8,path9,path10];
       
    folder_path = '/home/cesare/TESI/Path/';
    save([folder_path 'path_Start_Goal' num2str(i)],'path_vector')
    savefig([folder_path 'figure_StartGoal_' num2str(i)])
end