classdef ATRobotCheckerWithMovingObstacle < ATRobotChecker
    
    properties
        cyl_len;
        cyl_radius;
        xcyl;
        ycyl;
        zcyl;
        moving_obstacle;
    end
    
    methods
        function r = init(obj)
            init@ATRobotChecker(obj);
            
            obj.cyl_len =1.4;
            obj.cyl_radius = 0.2;
            obj.xcyl = 1;
            obj.ycyl = 0.7;
            obj.zcyl = obj.cyl_len/2;
            
            obj.moving_obstacle=collisionCylinder(obj.cyl_radius,obj.cyl_len);
            T=eye(4);
            T(1,4)=obj.xcyl;
            T(2,4)=obj.ycyl;
            T(3,4)= obj.zcyl;
            obj.moving_obstacle.Pose=T;
        end
        
        function r = check(obj,position)
            
            if length(position)~=3
                error('this example works only in 3d');
            end
            for idx=1:length(position)
                obj.conf(idx).JointPosition=position(idx);
            end
            
            obj.col0.Pose=obj.rbtree.getTransform(obj.conf,'link_0_col');
            obj.col1.Pose=obj.rbtree.getTransform(obj.conf,'link_1_col');
            obj.col2.Pose=obj.rbtree.getTransform(obj.conf,'link_2_col');
            obj.col3.Pose=obj.rbtree.getTransform(obj.conf,'link_3_col');
            obj.col4.Pose=obj.rbtree.getTransform(obj.conf,'tool_base_col');
            obj.col5.Pose=obj.rbtree.getTransform(obj.conf,'tool_col1');
            obj.col6.Pose=obj.rbtree.getTransform(obj.conf,'tool_col2');
            
            incube=~checkCollision(obj.col0,obj.obstacle) && ~checkCollision(obj.col1,obj.obstacle) && ~checkCollision(obj.col2,obj.obstacle) && ~checkCollision(obj.col3,obj.obstacle) && ~checkCollision(obj.col4,obj.obstacle) && ~checkCollision(obj.col5,obj.obstacle) && ~checkCollision(obj.col6,obj.obstacle);
            incyl=~checkCollision(obj.col0,obj.moving_obstacle) && ~checkCollision(obj.col1,obj.moving_obstacle) && ~checkCollision(obj.col2,obj.moving_obstacle) && ~checkCollision(obj.col3,obj.moving_obstacle) && ~checkCollision(obj.col4,obj.moving_obstacle) && ~checkCollision(obj.col5,obj.moving_obstacle) && ~checkCollision(obj.col6,obj.moving_obstacle);
            r = incube && incyl; 
        end
        
        function plot3d(obj,configuration)
            plot3d@ATRobotChecker(obj,configuration);
            ish=ishold(gca);
            if length(configuration)~=3
                error('this example works only in 3d');
            end
            
            hold on
            show(obj.moving_obstacle)
            xlim([-2 2])
            ylim([-2 2])
            zlim([-2 2])
            
            if ~ish
                hold off
            end
            
        end
        
        
        function plot(obj)
            
        end
    end
end