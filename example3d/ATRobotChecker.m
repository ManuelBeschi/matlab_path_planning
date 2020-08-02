classdef ATRobotChecker < CollisionChecker
    
    properties
        rbtree
        col0
        col1
        col2
        col3
        col4
        col5
        col6
        obstacle
        conf
    end
    
    methods
        function r = init(obj)
            init@CollisionChecker(obj,'example',1);
            obj.min_distance=0.01; % se no è troppo lento
            obj.rbtree = rigidBodyTree;
            
            rotx = @(t) [1 0 0; 0 cos(t) -sin(t) ; 0 sin(t) cos(t)] ;
            roty = @(t) [cos(t) 0 sin(t) ; 0 1 0 ; -sin(t) 0  cos(t)] ;
            rotz = @(t) [cos(t) -sin(t) 0 ; sin(t) cos(t) 0 ; 0 0 1] ;
            
            obs_size1=0.5;
            obs_size2=0.5;
            obs_size3=1.2;
            obj.obstacle=collisionBox(obs_size1,obs_size2,obs_size3);
            T=eye(4);
            T(1,4)=0.8;
            T(2,4)=1;
            T(3,4)= obs_size3/2;
            obj.obstacle.Pose=T;
            
            %show(col_obstacle)
            %hold on
            
            jnt0_len = 1;
            jnt1_len=0.6;
            jnt2_len=0.4;
            jnt3_len = 0.4;
            jnt_radius = 0.05;
            
            tool_len_base = 0.015 ;
            tool_radius_base = 0.06 ;
            tool_len = 0.12;
            tool_size = 0.02;
            
            
            body0 = rigidBody('link_0');
            jnt0 = rigidBodyJoint('jnt0','fixed');
            body0.Joint = jnt0;
            addBody(obj.rbtree,body0,obj.rbtree.BaseName);
            
            body0_col = rigidBody('link_0_col');
            jnt0_f = rigidBodyJoint('joint_0_f','fixed');
            T=eye(4,4);
            T(3,4)=0.5*jnt0_len;
            jnt0_f.setFixedTransform(T);
            body0_col.Joint = jnt0_f;
            addBody(obj.rbtree,body0_col,body0.Name);
            obj.col0=collisionCylinder(jnt_radius,jnt0_len);
            
            body1 = rigidBody('link_1');
            jnt1 = rigidBodyJoint('joint_1','revolute');
            T=eye(4,4);
            T(3,4)=jnt0_len;
            jnt1.setFixedTransform(T);
            body1.Joint = jnt1;
            addBody(obj.rbtree,body1,body0.Name);
            
            body1_col = rigidBody('link_1_col');
            jnt1_f = rigidBodyJoint('joint_1_f','fixed');
            T=eye(4,4);
            T(1:3,1:3)=rotx(-pi/2);
            T(2,4)=jnt1_len*0.5;
            jnt1_f.setFixedTransform(T);
            body1_col.Joint = jnt1_f;
            addBody(obj.rbtree,body1_col,body1.Name);
            obj.col1=collisionCylinder(jnt_radius,jnt1_len);
            
            body2 = rigidBody('link_2');
            jnt2 = rigidBodyJoint('joint_2','revolute');
            T=eye(4,4);
            T(1:3,1:3)=roty(pi/2);
            T(2,4)=jnt1_len;
            jnt2.setFixedTransform(T);
            body2.Joint = jnt2;
            addBody(obj.rbtree,body2,body1.Name);
            
            
            body2_col = rigidBody('link_2_col');
            jnt2_f = rigidBodyJoint('joint_2_f','fixed');
            T=eye(4,4);
            T(1:3,1:3)=rotx(pi/2);
            T(2,4)=jnt2_len*0.5;
            jnt2_f.setFixedTransform(T);
            body2_col.Joint = jnt2_f;
            addBody(obj.rbtree,body2_col,body2.Name);
            obj.col2=collisionCylinder(jnt_radius,jnt2_len);
            
            body3 = rigidBody('link_3');
            jnt3 = rigidBodyJoint('joint_3','revolute');
            T=eye(4,4);
            T(1:3,1:3)=roty(-pi/2);
            T(2,4)=jnt2_len;
            jnt3.setFixedTransform(T);
            body3.Joint = jnt3;
            addBody(obj.rbtree,body3,body2.Name);
            
            
            body3_col = rigidBody('link_3_col');
            jnt3_f = rigidBodyJoint('joint_3_f','fixed');
            T=eye(4,4);
            T(1:3,1:3)=rotx(-pi/2);
            T(2,4)=jnt3_len*0.5;
            jnt3_f.setFixedTransform(T);
            body3_col.Joint = jnt3_f;
            addBody(obj.rbtree,body3_col,body3.Name);
            obj.col3=collisionCylinder(jnt_radius,jnt3_len);
            
            tool_base = rigidBody('tool_base');
            jnt1_tool = rigidBodyJoint('joint_1_tool','fixed');
            T=eye(4,4);
            T(2,4)=jnt3_len;
            jnt1_tool.setFixedTransform(T);
            tool_base.Joint = jnt1_tool;
            addBody(obj.rbtree,tool_base,body3.Name);
            
            tool_base_col = rigidBody('tool_base_col');
            jnt1_tool_f = rigidBodyJoint('joint_1_tool_f','fixed');
            T=eye(4,4);
            T(1:3,1:3)=rotx(-pi/2);
            T(2,4)=tool_len_base*0.5;
            jnt1_tool_f.setFixedTransform(T);
            tool_base_col.Joint = jnt1_tool_f;
            addBody(obj.rbtree,tool_base_col,tool_base.Name);
            obj.col4=collisionCylinder(tool_radius_base,tool_len_base);
            
            tool = rigidBody('tool');
            jnt2_tool = rigidBodyJoint('joint_2_tool','fixed');
            T=eye(4,4);
            T(2,4)=tool_len_base;
            jnt2_tool.setFixedTransform(T);
            tool.Joint = jnt2_tool;
            addBody(obj.rbtree,tool,tool_base.Name);
            
            tool_col1 = rigidBody('tool_col1');
            jnt3_tool_f = rigidBodyJoint('joint_3_tool_f','fixed');
            T=eye(4,4);
            T(1:3,1:3)=rotx(-pi/2);
            T(2,4) = tool_len*0.5;
            T(3,4)=tool_radius_base-(tool_size/2);
            jnt3_tool_f.setFixedTransform(T);
            tool_col1.Joint = jnt3_tool_f;
            addBody(obj.rbtree,tool_col1,tool.Name);
            obj.col5=collisionBox(tool_size,tool_size,tool_len);
            
 
            tool_col2 = rigidBody('tool_col2');
            jnt4_tool_f = rigidBodyJoint('joint_4_tool_f','fixed');
            T=eye(4,4);
            T(1:3,1:3)=rotx(-pi/2);
            T(2,4) = tool_len*0.5;
            T(3,4)=-(tool_radius_base-(tool_size/2));
            jnt4_tool_f.setFixedTransform(T);
            tool_col2.Joint = jnt4_tool_f;
            addBody(obj.rbtree,tool_col2,tool.Name);
            obj.col6=collisionBox(tool_size,tool_size,tool_len);
            
            obj.conf=obj.rbtree.homeConfiguration;
            
            obj.col0.Pose=obj.rbtree.getTransform(obj.conf,'link_0_col');
            obj.col1.Pose=obj.rbtree.getTransform(obj.conf,'link_1_col');
            obj.col2.Pose=obj.rbtree.getTransform(obj.conf,'link_2_col');
            obj.col3.Pose=obj.rbtree.getTransform(obj.conf,'link_3_col');
            obj.col4.Pose=obj.rbtree.getTransform(obj.conf,'tool_base_col');
            obj.col5.Pose=obj.rbtree.getTransform(obj.conf,'tool_col1');
            obj.col6.Pose=obj.rbtree.getTransform(obj.conf,'tool_col2');
            
        end
        
        function r = check(obj,position) %1 if free, 0 if in collision
            
            
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
            
            r=~checkCollision(obj.col0,obj.obstacle) && ~checkCollision(obj.col1,obj.obstacle) && ~checkCollision(obj.col2,obj.obstacle) && ~checkCollision(obj.col3,obj.obstacle) && ~checkCollision(obj.col4,obj.obstacle) && ~checkCollision(obj.col5,obj.obstacle) && ~checkCollision(obj.col6,obj.obstacle);
        end
        
        function plot3d(obj,configuration)
            
            ish=ishold(gca);
            if length(configuration)~=3
                error('this example works only in 3d');
            end
            for idx=1:length(configuration)
                obj.conf(idx).JointPosition=configuration(idx);
            end
            
            obj.col0.Pose=obj.rbtree.getTransform(obj.conf,'link_0_col');
            obj.col1.Pose=obj.rbtree.getTransform(obj.conf,'link_1_col');
            obj.col2.Pose=obj.rbtree.getTransform(obj.conf,'link_2_col');
            obj.col3.Pose=obj.rbtree.getTransform(obj.conf,'link_3_col');
            obj.col4.Pose=obj.rbtree.getTransform(obj.conf,'tool_base_col');
            obj.col5.Pose=obj.rbtree.getTransform(obj.conf,'tool_col1');
            obj.col6.Pose=obj.rbtree.getTransform(obj.conf,'tool_col2');
            
            show(obj.col0)
            hold on 
            show(obj.col1)
            show(obj.col2)
            show(obj.col3)
            show(obj.col4)
            show(obj.col5)
            show(obj.col6)
            show(obj.obstacle)
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