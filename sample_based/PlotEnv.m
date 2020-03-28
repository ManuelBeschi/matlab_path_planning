path1 = current_path;
if(isequal(admissible_current_path,[]))
    path2 = reset_other_paths(1);
    path3 = reset_other_paths(2);
else
    path2 = reset_other_paths(2);
    path3 = reset_other_paths(3);
end
path1_nodes = path1.getWaypoints;
path2_nodes = path2.getWaypoints;
path3_nodes = path3.getWaypoints;
joints=replanned_path.getWaypoints;
nodePlot = path1_node_vector(j).q;

clf

hold on
obstacle='snowman';
checker=Snowman3dCollisionChecker;
checker.init;
checker.plot
metrics=Metrics;
view(135,0)
lb=-pi*ones(3,1);
ub=pi*ones(3,1);
if strcmp(obstacle,'torus')
    start_conf = [0 1.1 0]';
    goal_conf = [0.6 -0.7 0]';
else
    start_conf = [0.0 -1.1 0]';
    goal_conf =  [0.0 1.1 0]';
end
plot3(start_conf(1),start_conf(2),start_conf(3),'sy','MarkerFaceColor','b','MarkerSize',5)
plot3(goal_conf(1),goal_conf(2),goal_conf(3),'oy','MarkerFaceColor','r','MarkerSize',5)
axis equal
xlabel('q1');
ylabel('q2');
zlabel('q3');
plot3(path1_nodes(1,:)',path1_nodes(2,:)',path1_nodes(3,:)','--*b','LineWidth',0.5)
plot3(path2_nodes(1,:)',path2_nodes(2,:)',path2_nodes(3,:)','--*r','LineWidth',0.5)
plot3(path3_nodes(1,:)',path3_nodes(2,:)',path3_nodes(3,:)','--*g','LineWidth',0.5)
plot3(joints(1,:)',joints(2,:)',joints(3,:)','-y','LineWidth',1)
plot3(joints(1,:)',joints(2,:)',joints(3,:)','oy','LineWidth',1)
plot3(q(1,:)',q(2,:)',q(3,:)','*c','LineWidth',2)
plot3(nodePlot(1,:)',nodePlot(2,:)',nodePlot(3,:)','sr','LineWidth',2)
if(~isempty(examined_nodes))
    plot3(examined_nodes_plot(1,:)',examined_nodes_plot(2,:)',examined_nodes_plot(3,:)','ok','LineWidth',1);
end
