clearvars; close all; clc; warning off;

%20 coppie start-goal, 10 paths fissi per ognuna di esse. Ad ogni
%iterazione una coppia start-goal viene prelevata e un replanning per
%ognuno dei 10 paths viene eseguito, ognivolta scegliendo un path diverso
%come current_path. In totale 200 iterazioni per valutare il costo della
%soluzione trovata.

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

color = ["blue","red","green","black","white","#0072BD","#EDB120","#7E2F8E","#77AC30","#A2142F"];

start_conf = [start1,start2,start3,start4,start5,start6,start7,start8,start9,start10,start11,start12,start13,start14,start15,start16,start17,start18,start19,start20];
goal_conf = [goal1,goal2,goal3,goal4,goal5,goal6,goal7,goal8,goal9,goal10,goal11,goal12,goal13,goal14,goal15,goal16,goal17,goal18,goal19,goal20];

test_name = ['test_' num2str(succ_node) num2str(informed)];
mkdir(['/home/cesare/TESI/test_bonta_soluzione/' test_name]);

for j = 1:1:length(start_conf(1,:))
    close all
    load( ['/home/cesare/TESI/Path/path_Start_Goal' num2str(j)],'path_vector');
    
    folder_path = ['/home/cesare/TESI/test_bonta_soluzione/' test_name '/startGoal_' num2str(j)];
    mkdir(folder_path);
   
    time = [];
    time_vector = [];
    time_first_sol_vector = [];
    
    for i=1:1:length(path_vector)
        close all
        %% Figure
        
        fig = figure(i);
        checker.plot
        
        view(135,0)
        
        lb=-pi*ones(3,1);
        ub=pi*ones(3,1);
        
        hold on
        
        axis equal
        xlabel('q1');
        ylabel('q2');
        zlabel('q3');
        

        for k=1:length(path_vector)
        joints=path_vector(k).getWaypoints;
        colore = color(k);
        plot3(joints(1,:)',joints(2,:)',joints(3,:)','--','Color',colore,'LineWidth',1)
        end
 
        plot3(start_conf(1,j),start_conf(2,j),start_conf(3,j),'sy','MarkerFaceColor','c','MarkerSize',4)
        plot3(goal_conf(1,j),goal_conf(2,j),goal_conf(3,j),'oy','MarkerFaceColor','c','MarkerSize',4)
        %% Replan
        current_path = path_vector(i);
        other_paths = setdiff(path_vector,current_path);
        
        idx_replan=round(length(current_path.connections)*0.5);
        child=current_path.connections(:,idx_replan).getChild;
        parent=current_path.connections(:,idx_replan).getParent;
        q = (child.q+parent.q)/2;
        cost1 = current_path.getSubpathFromNode(child).cost;
        cost2 = metrics.cost(q,child.q);
        cost_original = cost1+cost2;
        old_cost = current_path.connections(:,end).getCost;
        current_path.connections(:,end).setCost(100);
        
        plot3(q(1),q(2),q(3),'oc','MarkerFaceColor','c','MarkerSize',4)
        
        tic
        [replanned_path,replanned_path_cost,success,replanned_path_vector,number_replanning,time_first_sol] = InformedOnlineReplanning(current_path,other_paths,q,lb,ub,max_distance,checker,metrics,opt_type,succ_node,informed,verbose);
        time = toc;
        
        time_vector = [time_vector,time]; %#ok<*AGROW>
        time_first_sol_vector = [time_first_sol_vector,time_first_sol]; %#ok<*AGROW>
        disp(['success: ' num2str(success) ' couple s-g n: ' num2str(j) ' starting path : ' num2str(i) ' time: ' num2str(toc) ' nodes:' num2str(number_replanning) ' cost:' num2str(replanned_path_cost)]);
        
        vector = [j i time time_first_sol number_replanning replanned_path_cost cost_original];
        matrix = [matrix; vector];
        
        if(isa(replanned_path,'Path'))
            joints=replanned_path.getWaypoints;
            plot3(joints(1,:)',joints(2,:)',joints(3,:)','-y','LineWidth',1)

        else
            warning('replanning not possible!');
        end
        
        current_path.connections(:,end).setCost(old_cost);
        
        %% Saving
        folder_name = [folder_path '/startingPath_' num2str(i)];
        save(folder_name)
        savefig(folder_name)
    end
end