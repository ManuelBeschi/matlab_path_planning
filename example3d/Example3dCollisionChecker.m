classdef Example3dCollisionChecker < CollisionChecker
    

    methods
        function r = init(obj)
            init@CollisionChecker(obj,'example',1);
        end
        
        function r = check(obj,position)
            if length(position)~=3
                error('this example works only in 3d');
            end
            r=any((abs(position))>1);
        end
    end
end

