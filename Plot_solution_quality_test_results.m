clearvars; close all; clc;
P = [];
P2 = [];
figure(1)
for j=1:1:6
    clearvars -except j P P2; clc;
    
    tests = [0 0 0 1 1 1; 0 1 2 0 1 2];
    color = ["red","black","green","yellow","cyan","blue"];
    total_matrix= [];
    
    succ_node = tests(1,j);
    informed = tests(2,j);
    
    folder_name = ['test_' num2str(succ_node)   num2str(informed)];
    folder_path = ['/home/cesare/TESI/test_bonta_soluzione/' folder_name];
    
    for i=1:1:20
        folder_path2 = [folder_path '/startGoal_' num2str(i)];
        file_name = [folder_path2 '/startingPath_10.mat'];
        load(file_name,'matrix');
        total_matrix = [total_matrix;matrix]; %#ok<*AGROW>      
    end

    y = (-total_matrix(:,7)+total_matrix(:,6))./total_matrix(:,7);
    x = total_matrix(:,3);
    
    figure(1)
    hold on
    [p1,p2] = plotPercentile(x,y,1,color(j));
    P = [P,p1];
    
    figure(2)
    hold on
    [p1,p2] = plotPercentile(j,y,2,color(j));
    P2 = [P2,p1];

end

figure(1)
title('Algorithms comparison','FontSize',18)
lgd=legend ([P(1) P(2) P(3) P(4) P(5) P(6)],{'00','01','02','10','11','12'},'FontSize',18);
title(lgd,'Algorithm version','FontSize',18)
xlabel('Time [s]','FontSize',18);
ylabel('(newCost - oldCost) / oldCost','FontSize',18);
axis([-0.1 27 -0.2 0.4])
grid on
savefig(['/home/cesare/TESI/test_bonta_soluzione/' 'Solution_quality_and_time'])

figure(2)
title('Algorithms comparison: Solution quality','FontSize',18)
lgd=legend ([P2(1) P2(2) P2(3) P2(4) P2(5) P2(6)],{'00','01','02','10','11','12'},'FontSize',18);
title(lgd,'Algorithm version','FontSize',18)
xlabel('Algorithm version','FontSize',18);
ylabel('(newCost - oldCost) / oldCost','FontSize',18);
axis([0 8 -0.2 0.4])
grid on
savefig(['/home/cesare/TESI/test_bonta_soluzione/' 'Solution_quality'])
