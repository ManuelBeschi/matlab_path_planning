classdef Torus3dCollisionChecker < CollisionChecker
    
    
    
    properties
        rp=0.9;
        rc=0.6;
        xc;
        
    end

    methods
        function r = init(obj)
            obj.xc=[0 0 0]';
            init@CollisionChecker(obj,'example',1);
        end
        
        function r = check(obj,position)
            if length(position)~=3
                error('this example works only in 3d');
            end
            x=position-obj.xc;
            r=(obj.rp-sqrt(x(1)^2+x(3)^2))^2+x(2)^2>obj.rc^2;
        end
        
        function plot(obj)
            
            drawTorus([obj.xc' obj.rp obj.rc 90 90],'facecolor','k','FaceAlpha',0.5)
            
        end
    end
end

