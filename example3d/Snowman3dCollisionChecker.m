classdef Snowman3dCollisionChecker < CollisionChecker
    

    properties
        rp=1;
        rc=0.5;
        xp;
        xc;
        
    end
    methods
        function r = init(obj)
            init@CollisionChecker(obj,'example',1);
            obj.xp=[0;0;0];
            obj.xc=[0 0 0.9*(obj.rp+obj.rc)];
            
        end
        
        function r = check(obj,position)
            if length(position)~=3
                error('this example works only in 3d');
            end
            r=norm(position-obj.xp)>obj.rp && norm(position-obj.xc)>obj.rc;
        end
        
        function plot(obj)
            xp=obj.xp;
            rp=obj.rp;
            xc=obj.xc;
            rc=obj.rc;
            ish=ishold;
            hold on
            [x,y,z] = ellipsoid(xp(1),xp(2),xp(3),rp,rp,rp);
            s=surf(x,y,z,'FaceAlpha',0.5);
            [x,y,z] = ellipsoid(xc(1),xc(2),xc(3),rc,rc,rc);
            s=surf(x,y,z,'FaceAlpha',0.5);
            if ~ish
                hold off;
            end
        end
    end
end

