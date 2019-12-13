classdef Cubes3dCollisionChecker < CollisionChecker
    
    properties
        centers
        widths;
        ncubes;
    end
    
    methods
        function r = init(obj)
            init@CollisionChecker(obj,'example',1);
            obj.ncubes=50;
            for i=1:obj.ncubes
                obj.centers(:,i)=-2+4*rand(3,1);
%                 obj.widths(:,i)=-.5+1*rand(3,1);
                obj.widths(:,i)=[1 1 1]*0.4;
            end
            r=1;
        end
        
        function r = check(obj,position)
            if length(position)~=3
                error('this example works only in 3d');
            end
            r=1;
            for i=1:obj.ncubes
                r=r &&any(abs(position-obj.centers(:,i))>0.5*obj.widths(:,i));
                if ~r
                    return
                end
            end
        end
        
        function plot(obj)
            for i=1:obj.ncubes
%                 plotcube((obj.centers(:,i)-obj.widths(:,i)*0.5)',(obj.centers(:,i)+obj.widths(:,i)*0.5)',0.5,[0 0 0])
                plotcube(obj.widths(:,i)',(obj.centers(:,i)-obj.widths(:,i)*0.5)',0.5,[0 0 0])
            end
        end
    end
end
