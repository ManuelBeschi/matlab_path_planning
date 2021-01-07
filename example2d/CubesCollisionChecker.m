classdef CubesCollisionChecker < CollisionChecker
    
    
    
    properties
        cube_distance=1;
        cube_width=0.7;
        lb=[-5;-4];
        ub=[5;4]
        cubes
        cube_centers;
    end
    
    methods
        function r = init(obj,lb,ub)
            init@CollisionChecker(obj,'example',1);
            obj.lb=lb;
            obj.ub=ub;
            xc=round(obj.lb(1)/obj.cube_distance:obj.ub(1)/obj.cube_distance)'*obj.cube_distance;
            yc=round(obj.lb(2)/obj.cube_distance:obj.ub(2)/obj.cube_distance)'*obj.cube_distance;
            [X,Y]=meshgrid(xc,yc);
            obj.cube_centers=[X(:),Y(:)];
            for ic=1:size(obj.cube_centers,1)
                obj.cubes(ic,:)=[obj.cube_centers(ic,1)-0.5*obj.cube_width obj.cube_centers(ic,2)-0.5*obj.cube_width obj.cube_width obj.cube_width];
            end
        end
        
        function r = check(obj,position)
            if length(position)~=2
                error('this example works only in 2d');
            end
            x1=mod(position(1),obj.cube_distance);
            x2=mod(position(2),obj.cube_distance);
            r=~and(or(x1<=obj.cube_width*0.5,x1>=obj.cube_distance-obj.cube_width*0.5),or(x2<=obj.cube_width*0.5,x2>=obj.cube_distance-obj.cube_width*0.5));
        end
        
        function plot(obj)
            for ic=1:size(obj.cube_centers,1)
                %rectangle('Position',obj.cubes(ic,:),'EdgeColor','k')
                rectangle('Position',obj.cubes(ic,:),'FaceColor',[1 1 1]*0.5,'EdgeColor',[1 1 1]*0.5)
            end
        end
    end
end

