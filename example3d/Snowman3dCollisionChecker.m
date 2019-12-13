classdef Snowman3dCollisionChecker < CollisionChecker
    

    properties
        rp=1;
        rc=0.5;
        rb=0.3;
        xp;
        xc;
        
        xb1
        xb2
    end
    methods
        function r = init(obj)
            init@CollisionChecker(obj,'example',1);
            obj.xp=[0;0;0];
            obj.xc=[0 0 0.9*(obj.rp+obj.rc)]';
            angl=45;
            obj.xb1=[0 (obj.rp+0*obj.rb)*cosd(angl) (obj.rp+0*obj.rb)*sind(angl)]';
            obj.xb2=[0 -(obj.rp+0*obj.rb)*cosd(angl) (obj.rp+0*obj.rb)*sind(angl)]';
        end
        
        function r = check(obj,position)
            if length(position)~=3
                error('this example works only in 3d');
            end
            r=norm(position-obj.xp)>obj.rp && norm(position-obj.xc)>obj.rc && norm(position-obj.xb1)>obj.rb && norm(position-obj.xb2)>obj.rb;
        end
        
        function plot(obj)
            xp=obj.xp;
            rp=obj.rp;
            xc=obj.xc;
            rc=obj.rc;
            xb1=obj.xb1;
            xb2=obj.xb2;
            rb=obj.rb;
            ish=ishold;
            hold on
            alpha=1.0;
            [x,y,z] = ellipsoid(xp(1),xp(2),xp(3),rp,rp,rp);
            s=surf(x,y,z,'FaceAlpha',alpha,'FaceColor',[1 1 1]*0.5);
            
            [x,y,z] = ellipsoid(xc(1),xc(2),xc(3),rc,rc,rc);
            s=surf(x,y,z,'FaceAlpha',alpha,'FaceColor',[1 1 1]*0.5);
            [x,y,z] = ellipsoid(xb1(1),xb1(2),xb1(3),rb,rb,rb);
            s=surf(x,y,z,'FaceAlpha',alpha,'FaceColor',[1 1 1]*0.5);
            [x,y,z] = ellipsoid(xb2(1),xb2(2),xb2(3),rb,rb,rb);
            s=surf(x,y,z,'FaceAlpha',alpha,'FaceColor',[1 1 1]*0.5);
            if ~ish
                hold off;
            end
        end
    end
end

