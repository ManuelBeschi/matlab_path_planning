clear all;close all;


filename = 'goals5000_t1';

baseline=0;
informed=true;
local_opt=false;

figure('Position',[10 10 1200 600])
axis equal
xlabel('q1');
ylabel('q2');


lb=-pi*ones(2,1);
ub=pi*ones(2,1);

checker=CubesCollisionChecker;
checker.init(lb,ub);
checker.plot
hold on
box on
axis equal
axis square

start_conf = [0 0.5]';
if ~checker.check(start_conf)
    error('start conf is in collision');
end
start_h=plot(start_conf(1),start_conf(2),'om','MarkerFaceColor','m','MarkerSize',15);

ntest=5000;
goals(:,1)=lb(1)+(ub(1)-lb(1))*rand(ntest*10,1);
goals(:,2)=lb(2)+(ub(2)-lb(2))*rand(ntest*10,1);
norm_goal=sqrt(goals(:,1).^2+goals(:,2).^2);
goals=goals(norm_goal>1.5,:);
goals=goals(1:ntest,:);

goal_h=[];
goal_valid=zeros(ntest,1);
for idx=1:ntest
    if checker.check(goals(idx,:))
        goal_h(idx,1)=plot(goals(idx,1),goals(idx,2),'ok','MarkerFaceColor','k');
        goal_valid(idx)=1;
    else
        goal_h(idx,1)=plot(goals(idx,1),goals(idx,2),'or','MarkerFaceColor','r');
    end
end
drawnow
export_fig([filename,'_all_goals.png'])

im= export_fig;
[imind,cm] = rgb2ind(im,256);
imwrite(imind,cm,[filename,'.gif'],'gif', 'Loopcount',inf);

delete(goal_h),clear goal_h
goals=goals(goal_valid==1,:);
ntest=size(goals,1);
for idx=1:ntest
    goal_h(idx,1)=plot(goals(idx,1),goals(idx,2),'ok','MarkerFaceColor','k');
end
drawnow
export_fig([filename,'_all_valid_goals.png'])
im= export_fig;
[imind,cm] = rgb2ind(im,256);
imwrite(imind,cm,[filename,'.gif'],'gif','WriteMode','append');


limx=xlim;
limy=ylim;
max_distance=0.1;
r_rewire=0.2;
metrics = Metrics;


start_tree=[];
for idx=1:ntest
    goal_conf = goals(idx,:)';
    sampler(idx) = InformedSampler(start_conf,goal_conf,lb,ub);
    if (idx>1)
        solver(idx) = BirrtConnect(solver(idx-1).start_tree,goal_conf,max_distance,checker,sampler(idx),metrics);
    else
        solver(idx) = BirrtConnect(start_conf,goal_conf,max_distance,checker,sampler(idx),metrics);
    end
    goal_node(idx)=solver(idx).goal_tree.root;
end

best_cost=inf;
path_h=gobjects(ntest,1);
eli_h=gobjects(ntest,1);
success=zeros(ntest,1);
discard_goal=zeros(ntest,1);
using_star=zeros(ntest,1);

rrtstar_idx=1;
idx_to_rrtstar_idx=[];

tree=solver(1).start_tree;
best_path_idx=[];

%%

for itrial=1:10%100
    for idx=1:ntest
        for istep=1:20
            
            if and(norm(goals(idx,:)'-start_conf)>best_cost,not(baseline))
                discard_goal(idx)=1;
            end
            if (discard_goal(idx))
                continue
            end
            if (using_star(idx)==0)
                [success(idx),path]=solver(idx).step;
            else
                improved=opt_solver(idx_to_rrtstar_idx(idx)).step;
                path=Path(tree.getConnectionToNode(goal_node(idx)));
                if not(improved)
                    continue;
                end
            end
            if ~success(idx)
                continue
            end
            
            if (best_cost>path.cost)
                best_path_idx=idx;
                best_cost=path.cost;
                for isolver=1:ntest
                    if baseline
                        sampler(isolver).setCost(inf);
                    else
                        sampler(isolver).setCost(best_cost);
                    end
                end
            end
            
            if (using_star(idx)==0)
                using_star(idx)=1;
                idx_to_rrtstar_idx(idx)=rrtstar_idx;
                opt_solver(rrtstar_idx)=RRTStar(solver(idx).start_tree,goal_node(idx),sampler(idx),checker,metrics,r_rewire);
                rrtstar_idx=rrtstar_idx+1;
            end
            
            
            fprintf('cost: cost=%f\n',path.cost);
        end
    end
    
    for isolver=1:ntest
        
        if and(norm(goals(idx,:)'-start_conf)>best_cost,not(baseline))
            discard_goal(idx)=1;
        end
        
        if ishandle(eli_h(isolver))
            delete(eli_h(isolver))
        end
        if ishandle(path_h(isolver))
            delete(path_h(isolver))
        end
        delete(goal_h(isolver,1))
        solver(isolver).goal_tree.deletePlot
    end
    
    
    tree.deletePlot
    delete(start_h)
    solver(idx).start_tree.plot
    start_h=plot(start_conf(1),start_conf(2),'om','MarkerFaceColor','m','MarkerSize',15);
    
    for isolver=1:ntest
        if ~discard_goal(isolver)
            eli_h(isolver)=sampler(isolver).plotEllipsoid;
            goal_h(isolver,1)=plot(goals(isolver,1),goals(isolver,2),'ok','MarkerFaceColor','k');
            
            if success(isolver)
                path=Path(tree.getConnectionToNode(goal_node(isolver)));
                joints=path.getWaypoints;
                if isolver==best_path_idx
                    path_h(isolver)=plot(joints(1,:)',joints(2,:)','b','LineWidth',4);
                else
                    path_h(isolver)=plot(joints(1,:)',joints(2,:)','b','LineWidth',1);
                end
            else
                solver(isolver).goal_tree.plot
            end
        else
            goal_h(isolver,1)=plot(goals(isolver,1),goals(isolver,2),'o','Color',[1 1 1]*0.8,'MarkerFaceColor',[1 1 1]*0.8);
        end
        
        
    end
    xlim(limx);
    ylim(limy);
    drawnow
    
    im= export_fig;
    [imind,cm] = rgb2ind(im,256);
    imwrite(imind,cm,[filename,'.gif'],'gif','WriteMode','append');
end

return

%%

for isolver=1:ntest
    if ~discard_goal(isolver)
        for idx=1:100
            s=sampler(isolver).sample;
            plot(s(1),s(2),'.k')
        end
    end
end