classdef birrt_extend < handle
    %BIRRT_EXTEND Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        start_conf
        goal_conf
        max_distance
        checker
        start_tree
        goal_tree
        sampler
    end
    
    methods
        function obj = birrt_extend(start_conf,goal_conf,max_distance,checker,sampler)
            obj.start_conf = start_conf;
            obj.goal_conf=goal_conf;
            obj.max_distance=max_distance;
            obj.checker=checker;
            
            
            start_node=node(start_conf);
            goal_node=node(goal_conf);
            obj.start_tree=tree(start_node,1,max_distance,checker);
            obj.goal_tree=tree(goal_node,0,max_distance,checker);
            
            obj.sampler=sampler;
        end
        
        function [solved,path] = step(obj)
            path=[];
            solved=false;
            q=obj.sampler.sample;
            [add_to_start,new_t1_node]=obj.start_tree.extend(q);
            if (add_to_start)
                [add_to_goal,new_t2_node]=obj.goal_tree.extendToNode(new_t1_node);
            else
                [add_to_goal,new_t2_node]=obj.goal_tree.extend(q);
            end
            if (add_to_goal && add_to_start && isequal(new_t1_node,new_t2_node))
                solved=true;
                path=[obj.start_tree.getConnectionToNode(new_t1_node) obj.goal_tree.getConnectionToNode(new_t1_node)];
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

