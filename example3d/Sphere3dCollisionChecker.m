classdef Sphere3dCollisionChecker < CollisionChecker
    

    methods
        function r = init(obj)
            init@CollisionChecker(obj,'example',1);
        end
        
        function r = check(obj,position)
            if length(position)~=3
                error('this example works only in 3d');
            end
            r=norm(position)>1;
        end
        
        function plot(obj)
            
            [x,y,z] = ellipsoid(0,0,0,1,1,1);
            s=surf(x,y,z,'FaceAlpha',0.5);
            
        end
    end
end

