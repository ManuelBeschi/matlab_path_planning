clearvars; close all; clc;
P = [];
figure(1)
for j=1:1:6
    clearvars -except j P; %close all; clc;
    
    tests = [0 0 0 1 1 1; 0 1 2 0 1 2];
    total_time= [];
    color = ["red","black","green","yellow","cyan","blue"];
    
    succ_node = tests(1,j);
    informed = tests(2,j);
    
    folder_name1 = ['test_' num2str(succ_node)  num2str(informed)];
    folder_path = ['/home/cesare/TESI/test_tempo_esecuzione/' folder_name1];
    
    for i=1:1:10
        folder_name2 = ['/test_' num2str(i) '_succNode_informed_' num2str(succ_node) num2str(informed)];
        folder_path2 = [folder_path folder_name2 '/test_number_20.mat'];
        load([folder_path folder_name2 '/test_number_20.mat'],'time_vector');
        total_time = [total_time,time_vector]; %#ok<*AGROW>
    end
    
%     standard_dev = std(total_time);
%     m = mean(total_time);
%    
%     histogram(total_time,1000)
%     title(['histogram:' num2str(succ_node) num2str(informed) '-> mean = ' num2str(m) ' standard deviation = ' num2str(standard_dev)])
%     xlabel('Execution Time[s]');
%     ylabel('Frequency');
%     save([folder_path '/total_time.mat'],'total_time');
%     savefig([folder_path '/histogram' num2str(succ_node) num2str(informed)])
%     pause

hold on
[p1,p2] = plotPercentile(j,total_time,1,color(j));
P = [P,p1];
end

title('Algorithms comparison')
legend ([P(1) P(2) P(3) P(4) P(5) P(6)],{'00','01','02','10','11','12'});
xlabel('Algorithm');
ylabel('Execution Time[s]');
axis([0 8 -1 28])
grid on
savefig(['/home/cesare/TESI/test_tempo_esecuzione/' 'Execution_Time'])
    