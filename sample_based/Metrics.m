classdef Metrics < handle
    %METRICS compute the cost between two node using a metrics
    %   In this class the cost is the euclidean norm. Its children can use
    %   weigth function.
    
    properties
    end
    
    methods
        function obj = Metrics()
        end
        
        function cost = cost(obj,configuration1,configuration2)
            if isa(configuration1,'Node')
                configuration1=configuration1.q;
            end
            if isa(configuration2,'Node')
                configuration2=configuration2.q;
            end
            
            cost=norm(configuration2-configuration1);
        end
    end
end

