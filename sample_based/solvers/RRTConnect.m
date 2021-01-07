classdef RRTConnect < Solver
    %BIRRT_EXTEND Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        start_conf
        goal_node
        max_distance
        start_tree
    end
    
    methods
        function obj = RRTConnect(start_conf,goal_conf,max_distance,checker,sampler,metrics)
            
            obj=obj@Solver(sampler,checker,metrics);
            obj.max_distance=max_distance;
            
            
            obj.checker=checker;
            obj.metrics=metrics;
            
            if isa(start_conf,'Tree')
                obj.start_conf=start_conf.root.q;
                obj.start_tree=start_conf;
            elseif isa(start_conf,'Node')
                obj.start_conf=start_conf.q;
                start_node=start_conf;
                obj.start_tree=Tree(start_node,1,max_distance,checker,metrics);
            else 
                obj.start_conf = start_conf;
                start_node=Node(start_conf);
                obj.start_tree=Tree(start_node,1,max_distance,checker,metrics);
            end
            
            if isa(goal_conf,'Node')
                obj.goal_node=goal_conf;
            else 
                obj.goal_node=Node(goal_conf);
            end
            
        end
        
        function [solved,path] = step(obj)
            path=[];
            solved=false;
            q=obj.sampler.sample;
            [add_to_start,new_t1_node]=obj.start_tree.connect(q);
            if add_to_start
                if norm(new_t1_node.q-obj.goal_node.q)<obj.max_distance
                    if obj.checker.checkPath([new_t1_node.q,obj.goal_node.q])
                        conn_cost=obj.metrics.cost(new_t1_node.q,obj.goal_node.q);
                        new_conn=Connection(new_t1_node,obj.goal_node,conn_cost);
                        solved=true;
                        connections=obj.start_tree.getConnectionToNode(obj.goal_node);
                        path=Path(connections);
                    end
                end
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

