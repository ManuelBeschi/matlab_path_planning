classdef Solver < handle
    %SOLVER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        sampler
        checker
        metrics
    end
    
    methods
        function obj = Solver(sampler,checker,metrics)
            %SOLVER Construct an instance of this class
            %   Detailed explanation goes here
            obj.sampler= sampler;
            obj.checker=checker;
            obj.metrics=metrics;
        end
        
        function [solved,path] = step(obj)
            %STEP one iteration of the algorithm
            %   ....
            solved=false;
            path=[];
        end
        
        function [solved,path] = solve(obj)
            %SOLVE run until find a solution
            %   ....
            solved=false;
            path=[];
        end
    end
end

