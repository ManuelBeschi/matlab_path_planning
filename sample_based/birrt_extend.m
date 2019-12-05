classdef birrt_extend < handle
    %BIRRT_EXTEND Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        start_conf
        goal_conf
        max_distance
        checker
    end
    
    methods
        function obj = birrt_extend(start_conf,goal_conf,max_distance,checker)
            obj.start_conf = start_conf;
            obj.goal_conf=goal_conf;
            obj.max_distance=max_distance;
            obj.checker=checker;
        end
        
        function outputArg = solve(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

