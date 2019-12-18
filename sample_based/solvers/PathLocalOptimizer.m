classdef PathLocalOptimizer < handle
    %SOLVER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        path
        type
        checker
        metrics
        stall_gen
        max_stall_gen
    end
    
    methods
        function obj = PathLocalOptimizer(path,type,checker,metrics)
            obj.path=path;
            obj.type=type;
            obj.checker=checker;
            obj.metrics=metrics;
            obj.stall_gen=0;
            obj.max_stall_gen=10;
        end
        
        function [solved,path] = step(obj)
            solved=true;
            path=obj.path;
            cost=path.cost;
            if strcmp(obj.type,'slip')
                solved = path.slipParent(obj.checker,obj.metrics) && solved;
                solved = path.slipChild(obj.checker,obj.metrics) && solved;
            elseif strcmp(obj.type,'slipParent')
                solved = path.slipParent(obj.checker,obj.metrics) && solved;
            elseif strcmp(obj.type,'slipChild')
                solved = path.slipChild(obj.checker,obj.metrics) && solved;
            elseif strcmp(obj.type,'warp')
                solved = path.warp(obj.checker,obj.metrics) && solved;
            elseif strcmp(obj.type,'spiral')
                solved = path.spiral(obj.checker,obj.metrics) && solved;
            elseif strcmp(obj.type,'full')
                solved = path.warp(obj.checker,obj.metrics) && solved;
                solved = path.spiral(obj.checker,obj.metrics) && solved;
                solved = path.slipParent(obj.checker,obj.metrics) && solved;
                solved = path.slipChild(obj.checker,obj.metrics) && solved;
            end
            if (cost<=path.cost)
                if (obj.stall_gen==0)
                    path.simplify(obj.checker,obj.metrics);
                end
                obj.stall_gen=obj.stall_gen+1;
            else
                obj.stall_gen=0;
            end
            if (obj.stall_gen>obj.max_stall_gen)
                solved=true;
            end
            
        end
        
        function [solved,path] = solve(obj,iter)
            if nargin<2
                iter=50;
            end
            solved=false;
            for idx=1:iter
                [solved,path] = step(obj);
                if solved
                    return;
                end
            end
        end
    end
end

