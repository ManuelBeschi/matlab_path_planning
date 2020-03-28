clearvars; close all; clc; warning off;

matrix = [];

succ_node = 1;
informed = 2;

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

for j = 1:1:10 %10
    close all
    
    folder_name = ['test_' num2str(j) '_succNode_informed_' num2str(succ_node) num2str(informed)];
    mkdir('/home/cesare/TESI/prova3/',folder_name);
    folder_path = ['/home/cesare/TESI/prova3/' folder_name];
   
    time = [];
    time_vector = [];
    
    for i=1:1:length(start_conf(1,:))
        close all
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
        
        %path1.connections(end).setCost(30); % NB: così sei sicuro che lo switch avverrà
        path1.connections(end).setCost(inf);
        
        joints=path1.getWaypoints;
        plot3(joints(1,:)',joints(2,:)',joints(3,:)','--b','LineWidth',1)
        
        %% 2° Path
        solver2 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
        [success,path2]=solver2.solve;
        path2=path2.resample(connection_max_length,metrics);
        
        path_optimizer=PathLocalOptimizer(path2,opt_type,checker,metrics);
        path_optimizer.solve;
        
        joints=path2.getWaypoints;
        plot3(joints(1,:)',joints(2,:)',joints(3,:)','--r','LineWidth',1)
        
        %% 3° Path
        solver3 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
        [success,path3]=solver3.solve;
        path3=path3.resample(connection_max_length,metrics);
        
        path_optimizer=PathLocalOptimizer(path3,opt_type,checker,metrics);
        path_optimizer.solve;
        
        joints=path3.getWaypoints;
        plot3(joints(1,:)',joints(2,:)',joints(3,:)','--g','LineWidth',1)
        
        %% 4° Path
        solver4 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
        [success,path4]=solver4.solve;
        path4=path4.resample(connection_max_length,metrics);
        
        path_optimizer=PathLocalOptimizer(path4,opt_type,checker,metrics);
        path_optimizer.solve;
        
        joints=path4.getWaypoints;
        plot3(joints(1,:)',joints(2,:)',joints(3,:)','--k','LineWidth',1)
        
        %% 5° Path
        solver5 = BirrtConnect(start_conf(:,i),goal_conf(:,i),max_distance,checker,sampler,metrics);
        [success,path5]=solver5.solve;
        path5=path5.resample(connection_max_length,metrics);
        
        path_optimizer=PathLocalOptimizer(path5,opt_type,checker,metrics);
        path_optimizer.solve;
        
        joints=path5.getWaypoints;
        plot3(joints(1,:)',joints(2,:)',joints(3,:)','--w','LineWidth',1)
        
        %% Replan
        current_path = path1;
        other_paths = [path2 path3 path4 path5];
        
        idx_replan=round(length(path1.connections)*0.5);
        child=current_path.connections(:,idx_replan).getChild.q;
        parent=current_path.connections(:,idx_replan).getParent.q;
        q = (child+parent)/2;
        %current_path.connections(:,idx_replan).setCost(inf);
        
        %  path1_nodes = path1.getWaypoints;
        %  path2_nodes = path2.getWaypoints;
        %  path3_nodes = path3.getWaypoints;
        %  plot3(q(1,:)',q(2,:)',q(3,:)','*c','LineWidth',2)
        %  plot3(path1_nodes(1,:)',path1_nodes(2,:)',path1_nodes(3,:)','*b','LineWidth',0.5)
        %  plot3(path2_nodes(1,:)',path2_nodes(2,:)',path2_nodes(3,:)','*r','LineWidth',0.5)
        %  plot3(path3_nodes(1,:)',path3_nodes(2,:)',path3_nodes(3,:)','*g','LineWidth',0.5)
        
        profile on
        tic
        [replanned_path,replanned_path_cost,success,replanned_path_vector,number_replanning] = InformedOnlineReplanning(current_path,other_paths,q,lb,ub,max_distance,checker,metrics,opt_type,succ_node,informed,verbose);
        time = toc;
        if(toc>40)
            profile viewer;
            pause;
        end
        profile off
        
        time_vector = [time_vector,time]; %#ok<*AGROW>
        disp(['success: ' num2str(success) ' test n: ' num2str(j) ' coppia s-g: ' num2str(i) ' time: ' num2str(toc) ' nodes:' num2str(number_replanning) ' cost:' num2str(replanned_path_cost)]);
        
        vector = [j i toc number_replanning replanned_path_cost];
        matrix = [matrix; vector];
        
        if(isa(replanned_path,'Path'))
            joints=replanned_path.getWaypoints;
            plot3(joints(1,:)',joints(2,:)',joints(3,:)','-y','LineWidth',1)
            %     plot3(joints(1,:)',joints(2,:)',joints(3,:)','oy','LineWidth',1)
            %     replanned_path.verboseDebug(false);
            %     path_optimizer=PathLocalOptimizer(replanned_path,opt_type,checker,metrics);
            %     path_optimizer.solve;
            %     joints=replanned_path.getWaypoints;
            %     plot3(joints(1,:)',joints(2,:)',joints(3,:)','-y','LineWidth',1.5)
        else
            warning('replanning not possible!');
        end
        
        save([folder_path '/test_number_' num2str(i)])
        savefig([folder_path '/figure_test_number' num2str(i)])
    end
end