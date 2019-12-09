classdef BirrtExtend < handle
    %BIRRT_EXTEND Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        start_conf
        goal_conf
        max_distance
        checker
        start_tree
        goal_tree
        start_node
        goal_node;
        sampler
    end
    
    methods
        function obj = BirrtExtend(start_conf,goal_conf,max_distance,checker,sampler)
            obj.max_distance=max_distance;
            obj.checker=checker;
            
            if isobject(start_conf)
                obj.start_conf=start_conf.q;
                obj.start_node=start_conf;
            else 
                obj.start_conf = start_conf;
                obj.start_node=Node(start_conf);
            end
            
            if isobject(goal_conf)
                obj.goal_conf=goal_conf.q;
                obj.goal_node=goal_conf;
            else 
                obj.goal_conf=goal_conf;
                obj.goal_node=Node(goal_conf);
            end
            
            obj.start_tree=Tree(obj.start_node,1,max_distance,checker);
            obj.goal_tree=Tree(obj.goal_node,0,max_distance,checker);
            
            obj.sampler=sampler;
        end
        
        function [solved,opt_path] = step(obj)
            opt_path=[];
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
                connections=[obj.start_tree.getConnectionToNode(new_t1_node) obj.goal_tree.getConnectionToNode(new_t1_node)];
                opt_path=Path(connections);
            end
        end
        
        function [solved,opt_path] = solve(obj)
            obj.start_tree=Tree(obj.start_node,1,obj.max_distance,obj.checker);
            obj.goal_tree=Tree(obj.goal_node,0,obj.max_distance,obj.checker);
            solved=false;
            for idx=1:1000
                [solved,opt_path]=obj.step;
                if (solved)
                    return;
                end
            end
        end
        
    end
end

