classdef BirrtConnect < Solver
    %BIRRT_EXTEND Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        start_conf
        goal_conf
        max_distance
        start_tree
        goal_tree
    end
    
    methods
        function obj = BirrtConnect(start_conf,goal_conf,max_distance,checker,sampler,metrics)
            
            obj=obj@Solver(sampler,checker,metrics);
            obj.max_distance=max_distance;
            
            
            obj.checker=checker;
            obj.metrics=metrics;
            
            if isobject(start_conf)
                obj.start_conf=start_conf.q;
                start_node=start_conf;
            else 
                obj.start_conf = start_conf;
                start_node=Node(start_conf);
            end
            
            if isobject(goal_conf)
                obj.goal_conf=goal_conf.q;
                goal_node=goal_conf;
            else 
                obj.goal_conf=goal_conf;
                goal_node=Node(goal_conf);
            end
            
            obj.start_tree=Tree(start_node,1,max_distance,checker,metrics);
            obj.goal_tree=Tree(goal_node,0,max_distance,checker,metrics);
            
            obj.sampler=sampler;
        end
        
        function [solved,path] = step(obj)
            path=[];
            solved=false;
            q=obj.sampler.sample;
            [add_to_start,new_t1_node]=obj.start_tree.connect(q);
            if (add_to_start)
                [add_to_goal,new_t2_node]=obj.goal_tree.connectToNode(new_t1_node);
            else
                [add_to_goal,new_t2_node]=obj.goal_tree.connect(q);
            end
            if (add_to_goal && add_to_start && isequal(new_t1_node,new_t2_node))
                solved=true;
                goal_subpath=obj.goal_tree.getConnectionToNode(new_t1_node);
                obj.goal_tree.keepOnlyThisBranch(goal_subpath);
                obj.start_tree.addBranch(goal_subpath);
                connections=obj.start_tree.getConnectionToNode(obj.goal_tree.root);
                path=Path(connections);
            end
        end
        
        function [solved,path] = solve(obj)
            solved=false;
            for idx=1:1000
                [solved,path]=obj.step;
                if (solved)
                    return;
                end
            end
        end
        
    end
end

