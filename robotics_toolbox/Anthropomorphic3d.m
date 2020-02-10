classdef Anthropomorphic3d < CollisionChecker
    
    properties
        rbtree
        col1
        col2
        obstacle
        conf
    end
    
    methods
        function r = init(obj)
            init@CollisionChecker(obj,'example',1);
            obj.min_distance=0.1; % se no è troppo lento
            obj.rbtree = rigidBodyTree;
            
            rotx = @(t) [1 0 0; 0 cos(t) -sin(t) ; 0 sin(t) cos(t)] ;
            roty = @(t) [cos(t) 0 sin(t) ; 0 1 0 ; -sin(t) 0  cos(t)] ;
            rotz = @(t) [cos(t) -sin(t) 0 ; sin(t) cos(t) 0 ; 0 0 1] ;
            
            obs_size=0.6;
            obj.obstacle=collisionBox(obs_size,obs_size,obs_size);
            T=eye(4);
            T(1:3,4)=0.5;
            obj.obstacle.Pose=T;
            
            %show(col_obstacle)
            %hold on
            
            jnt1_len=1;
            jnt1_siz=0.1;
            jnt2_len=1;
            jnt2_siz=0.1;
            
            
            
            bodybase = rigidBody('base_link');
            jnt0 = rigidBodyJoint('based_fixed','fixed');
            bodybase.Joint = jnt0;
            addBody(obj.rbtree,bodybase,obj.rbtree.BaseName);
            
            body0 = rigidBody('b0');
            jnt0 = rigidBodyJoint('jnt0','revolute');
            body0.Joint = jnt0;
            addBody(obj.rbtree,body0,bodybase.Name);
            
            
            body1 = rigidBody('link_1');
            jnt1 = rigidBodyJoint('joint_1','revolute');
            T=eye(4,4);
            T(1:3,1:3)=roty(-pi/2);
            T(3,4)=0;
            jnt1.setFixedTransform(T);
            body1.Joint = jnt1;
            addBody(obj.rbtree,body1,body0.Name);
            
            body1_col = rigidBody('link_1_col');
            jnt1_f = rigidBodyJoint('joint_1_f','fixed');
            T=eye(4,4);
            T(1,4)=jnt1_len*0.5;
            jnt1_f.setFixedTransform(T);
            body1_col.Joint = jnt1_f;
            addBody(obj.rbtree,body1_col,body1.Name);
            obj.col1=collisionBox(jnt1_len,jnt1_siz,jnt1_siz);
            
            
            Tjnt2=eye(4);
            Tjnt2(1,4)=1;
            
            body2 = rigidBody('link_2');
            jnt2 = rigidBodyJoint('joint_2','revolute');
            T=eye(4,4);
            T(1,4)=jnt1_len;
            jnt2.setFixedTransform(T);
            body2.Joint = jnt2;
            addBody(obj.rbtree,body2,body1.Name);
            
            
            body2_col = rigidBody('link_2_col');
            jnt2_f = rigidBodyJoint('joint_2_f','fixed');
            T=eye(4,4);
            T(1,4)=jnt2_len*0.5;
            T(3,4)=0.5*(jnt1_siz+jnt2_siz);
            jnt2_f.setFixedTransform(T);
            body2_col.Joint = jnt2_f;
            addBody(obj.rbtree,body2_col,body2.Name);
            obj.col2=collisionBox(jnt2_len,jnt2_siz,jnt2_siz);
            
            
            tool = rigidBody('tool');
            jnt2_tool = rigidBodyJoint('joint_2_tool','fixed');
            T=eye(4,4);
            T(1,4)=jnt2_len;
            T(3,4)=0.5*(jnt1_siz+jnt2_siz);
            jnt2_tool.setFixedTransform(T);
            body2_col.Joint = jnt2_tool;
            addBody(obj.rbtree,tool,body2.Name);
            
            
            
            obj.conf=obj.rbtree.homeConfiguration;
            
            obj.col1.Pose=obj.rbtree.getTransform(obj.conf,'link_1_col');
            obj.col2.Pose=obj.rbtree.getTransform(obj.conf,'link_2_col');
        
            
        end
        
        function r = check(obj,position)
            
            
            if length(position)~=3
                error('this example works only in 3d');
            end
            for idx=1:length(position)
                obj.conf(idx).JointPosition=position(idx);
            end
            
            obj.col1.Pose=obj.rbtree.getTransform(obj.conf,'link_1_col');
            obj.col2.Pose=obj.rbtree.getTransform(obj.conf,'link_2_col');
            r=~checkCollision(obj.col1,obj.obstacle) && ~checkCollision(obj.col2,obj.obstacle);
        end
        
        function plot3d(obj,configuration)
            
            ish=ishold(gca);
            if length(configuration)~=3
                error('this example works only in 3d');
            end
            for idx=1:length(configuration)
                obj.conf(idx).JointPosition=configuration(idx);
            end
            
            obj.col1.Pose=obj.rbtree.getTransform(obj.conf,'link_1_col');
            obj.col2.Pose=obj.rbtree.getTransform(obj.conf,'link_2_col');
            
            show(obj.col1)
            hold on
            show(obj.col2)
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
