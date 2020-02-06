classdef Snowman3dWithPoleCollisionChecker < Snowman3dCollisionChecker
    

    properties (Access = protected)
        xpole;
        rpole;
        zpole_min;
        zpole_max;
    end
    methods
        function r = init(obj)
            init@Snowman3dCollisionChecker(obj);
            obj.zpole_min=-1;
            obj.zpole_max=2;
            obj.xpole=[0 1.2 0];
            obj.rpole=0.25;
        end
        
        function r = check(obj,position)
            r=check@Snowman3dCollisionChecker(obj,position);
            if ~r
                return
            end
            
            incyl=(position(3)>=obj.zpole_min) && ...
                (position(3)<=obj.zpole_max) && ...
            (((position(1)-obj.xpole(1))^2+(position(2)-obj.xpole(2))^2)<=obj.rpole^2);
            r=r && ~incyl;
        
        end
        
        function plot(obj)
            plot@Snowman3dCollisionChecker(obj);
            ish=ishold;
            hold on
            x=obj.xpole;
            zmin=obj.zpole_min;
            zmax=obj.zpole_max;
            drawCylinder([x(1) x(2) zmin x(1) x(2) zmax obj.rpole])
            if ~ish
                hold off;
            end
        end
    end
end

