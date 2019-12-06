classdef InformedSampler < handle
    %INFORMED_SAMPLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        lb
        ub
        goal_conf
        start_conf
        best_path_cost
    end
    
    methods
        function obj = InformedSampler(start_conf,goal_conf,lb,ub,best_path_cost)
            obj.lb=lb;
            obj.ub=ub;
            
            if isobject(start_conf)
                obj.start_conf=start_conf.q;
            else
                obj.start_conf = start_conf;
            end
            
            if isobject(start_conf)
                obj.goal_conf=goal_conf.q;
            else
                obj.goal_conf=goal_conf;
            end
            if nargin>4
                obj.best_path_cost=best_path_cost;
            else
                obj.best_path_cost=inf;
            end
        end
        
        function q = sample(obj)
            q=obj.lb+(obj.ub-obj.lb).*rand(length(obj.lb),1);
        end
    end
end

