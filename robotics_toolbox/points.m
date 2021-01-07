clear all;close all;clc;

% questo file controlla che le configurazione presenti nel file
% configurations.mat siano corrette

% lower and upper bound
lb=-pi*ones(3,1);
ub=pi*ones(3,1);
% velocity bound
max_vel=2*ones(3,1);

load configurations.mat
% contiente la variabile tests che un vettore di una struttura con
% offset: posizione della base del robot 
% configurations: posizioni nello spazio dei giunti che devono essere
% raggiungere

metrics=Metrics;

for itest=1:length(tests)
    
    offset=tests(itest).offset; % posizione della base del robot in questo test
    goal_configurations=tests(itest).configurations;
    
    checker=Anthropomorphic3d;
    checker.init(lb,ub,offset);
    
    for iconf=1:size(goal_configurations,2)
        nodes(iconf) = Node(goal_configurations(:,iconf)); % punto di partenza nelle configurazioni
    end
    
    robot=checker.rbtree;
    
    for idx=1:length(nodes)
        if not(checker.check(nodes(idx).q))
            fprintf('Node %d is in conflict\n',idx)
        end
        checker.plot3d(nodes(idx).q)
        T_w_pose(:,:,idx)=robot.getTransform(nodes(idx).q,'tool');
        hold on
        plot3(T_w_pose(1,4,idx),T_w_pose(2,4,idx),T_w_pose(3,4,idx),'ok','MarkerFaceColor','k','MarkerSize',15)
        hold off
        drawnow
        pause(0.2)
    end
end
