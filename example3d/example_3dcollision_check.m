clear all;close all;clc;

checker=Example3dCollisionChecker;
checker.init;

%%
figure(1)

robot=importrobot('sharework_cell.urdf');
home_conf=homeConfiguration(robot);

lb=-pi*ones(3,1); % leggi da urdf
ub=pi*ones(3,1);


home = [1.1 0 0]';
down = [0 1.1 0]';

max_distance=0.5;

sampler = InformedSampler(home,down,lb,ub);
solver1 = BirrtConnect(home,down,max_distance,checker,sampler);
solver2 = BirrtExtend(home,down,max_distance,checker,sampler);

for itrial=1:10
    if itrial==1
        [success,path]=solver1.solve;
    else
        [success,path]=solver2.solve;
    end
    fprintf('cost=%f\n',path.cost);
    sampler.setCost(path.cost);
    
    
    if (~success)
        fprintf('failed');
        return
    end
    
    plotcube([2,2,2],[-1 -1 -1],0.5,[0 0 0])
    hold on
    sampler.plotEllipsoid
    joints=path.getWaypoints;
    plot3(joints(1,:)',joints(2,:)',joints(3,:)','LineWidth',2)
    plot3(home(1),home(2),home(3),'ob','MarkerFaceColor','b','MarkerSize',5)
    plot3(down(1),down(2),down(3),'or','MarkerFaceColor','r','MarkerSize',5)
    
    axis equal
    xlabel('q1');
    ylabel('q2');
    zlabel('q3');
    hold off
    
    pause
end
%%
