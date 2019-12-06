classdef informed_sampler < handle
    %INFORMED_SAMPLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        lb
        ub
    end
    
    methods
        function obj = informed_sampler(start_conf,goal_conf,lb,ub,best_path_cost)
            obj.lb=lb;
            obj.ub=ub;
        end
        
        function q = sample(obj)
            q=obj.lb+(obj.ub-obj.lb).*rand(length(obj.lb),1);
        end
    end
end

