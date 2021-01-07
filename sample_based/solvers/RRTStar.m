classdef RRTStar < Solver
    %RRTSTAR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        tree
        start_conf
        goal_node
        r_rewire
    end
    
    methods
        function obj = RRTStar(tree,goal_node,sampler,checker,metrics,r_rewire)
            obj=obj@Solver(sampler,checker,metrics);
           
            obj.tree=tree;
            obj.goal_node=goal_node;
            obj.r_rewire=r_rewire;
        end
        
        function improved=step(obj)
            improved=obj.tree.rewire(obj.sampler.sample,obj.checker,obj.r_rewire);
        end
        
        function [improved,path] = solve(obj)
            cost=obj.tree.costToNode(obj.goal_node);
            niter=1000;
            stall_gen=0;
            max_stall_gen=100;
            improved=false;
            path=Path(obj.tree.getConnectionToNode(obj.goal_node));
            informed=true;
            if informed
                obj.sampler.setCost(cost);
            end
            for it=1:niter
                obj.step;
                if cost>obj.tree.costToNode(obj.goal_node)
                    improved=true;
                    path=Path(obj.tree.getConnectionToNode(obj.goal_node));
                    cost=obj.tree.costToNode(obj.goal_node);
                    if informed
                        obj.sampler.setCost(cost);
                    end
                    stall_gen=0;
                else
                    stall_gen=stall_gen+1;
                    if stall_gen>max_stall_gen
                        return
                    end
                end
                
            end
        end
    end
end

