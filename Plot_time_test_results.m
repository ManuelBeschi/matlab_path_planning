clearvars; close all; clc;
P = [];
P_first_sol = [];
figure(1)
for j=1:1:6
    clearvars -except j P P_first_sol; %close all; clc;
    
    tests = [0 0 0 1 1 1; 0 1 2 0 1 2];
    total_time = [];
    total_time_first_sol = [];
    color = ["red","black","green","yellow","cyan","blue"];
    
    succ_node = tests(1,j);
    informed = tests(2,j);
    
    folder_name1 = ['test_' num2str(succ_node)  num2str(informed)];
    folder_path = ['/home/cesare/TESI/test_tempo_esecuzione/' folder_name1];
    
    for i=1:1:10
        folder_name2 = ['/test_' num2str(i) '_succNode_informed_' num2str(succ_node) num2str(informed)];
        folder_path2 = [folder_path folder_name2 '/test_number_20.mat'];
        load([folder_path folder_name2 '/test_number_20.mat'],'time_vector');
        load([folder_path folder_name2 '/test_number_20.mat'],'time_first_sol_vector');
        total_time = [total_time,time_vector]; %#ok<*AGROW>
        total_time_first_sol = [total_time_first_sol,time_first_sol_vector];%#ok<*AGROW>
    end
    
n1 = 1;
figure(n1)
hold on
[p1,p2] = plotPercentile(j,total_time,n1,color(j));
P = [P,p1];

n2 = 2;
figure(n2)
hold on
[p1_first_sol,p2_first_sol] = plotPercentile(j,total_time_first_sol,n2,color(j));
P_first_sol = [P_first_sol,p1_first_sol];
end

figure(n1)
title('Algorithms comparison: computational time','FontSize',18)
lgd=legend ([P(1) P(2) P(3) P(4) P(5) P(6)],{'00','01','02','10','11','12'},'FontSize',18);
title(lgd,'Algorithm version','FontSize',18)
xlabel('Algorithm version','FontSize',18);
ylabel('Computational Time[s]','FontSize',18);
axis([0 8 -1 28])
grid on
xticks([1 2 3 4 5 6])
%savefig(['/home/cesare/TESI/test_tempo_esecuzione/' 'Execution_Time'])

figure(n2)
title('Algorithms comparison: computational time to find a first solution','FontSize',18)
lgd=legend ([P_first_sol(1) P_first_sol(2) P_first_sol(3) P_first_sol(4) P_first_sol(5) P_first_sol(6)],{'00','01','02','10','11','12'},'FontSize',18);
title(lgd,'Algorithm version','FontSize',18)
xlabel('Algorithm version','FontSize',18);
ylabel('Computational Time[s]','FontSize',18);
axis([0 8 -0.1 1.2])
grid on
xticks([1 2 3 4 5 6])
%savefig(['/home/cesare/TESI/test_tempo_esecuzione/' 'Execution_Time_first_sol'])
