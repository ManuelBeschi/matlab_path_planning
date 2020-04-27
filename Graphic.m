clearvars; close all; clc;
P = [];
figure(1)
for j=1:1:6
    clearvars -except j P; clc;
    
    tests = [0 0 0 1 1 1; 0 1 2 0 1 2];
    color = ["red","black","green","yellow","cyan","blue"];
    total_matrix= [];
    
    succ_node = tests(1,j);
    informed = tests(2,j);
    
    folder_name = ['test_' num2str(succ_node)   num2str(informed)];
    folder_path = ['/home/cesare/TESI/test_bontaSoluzione/' folder_name];
    
    for i=1:1:20
        folder_path2 = [folder_path '/startGoal_' num2str(i)];
        file_name = [folder_path2 '/startingPath_10.mat'];
        load(file_name,'matrix');
        total_matrix = [total_matrix;matrix]; %#ok<*AGROW>      
    end

    y = (-total_matrix(:,6)+total_matrix(:,5))./total_matrix(:,6);
    x = total_matrix(:,3);
    
    hold on
    [p1,p2] = plotPercentile(x,y,1,color(j));
    P = [P,p1];
%     sty= std(y);
%     my = mean(y);
%     stx= std(x);
%     mx = mean(x);
%     
%     index = my+mx;
%     
%     figure
%     plot(x,y,'o')
%     title(['Graph:' num2str(succ_node) num2str(informed) '-> m_y = ' num2str(my) ' st_y = ' num2str(sty) ' m_x = ' num2str(mx) ' st_x = ' num2str(stx) ' index= ' num2str(index)])
%     save([folder_path '/total_matrix.mat'],'total_matrix');
%     savefig([folder_path '/graphic' num2str(succ_node) num2str(informed)])

end
 
title('Algorithms comparison')
legend ([P(1) P(2) P(3) P(4) P(5) P(6)],{'00','01','02','10','11','12'});
xlabel('Time [s]');
ylabel('(newCost - oldCost) / oldCost');
axis([-1 27 -0.05 0.4])
grid on