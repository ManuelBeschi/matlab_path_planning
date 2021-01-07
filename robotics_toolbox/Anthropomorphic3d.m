classdef Anthropomorphic3d < CollisionChecker
    
    properties
        rbtree
        cols
        col_names
        obstacles
        conf
        lb,
        ub
    end
    
    methods
        function r = init(obj,lb,ub,offset)
            if nargin<4
                offset=zeros(3,1);
            end
            if nargin<3
                obj.ub=pi*ones(3,1);
            else
                obj.ub=ub;
            end
            if nargin<2
                obj.bb=-pi*ones(3,1);
            else
                obj.lb=lb;
            end
            init@CollisionChecker(obj,'example',1);
            obj.min_distance=0.1; % se no è troppo lento
            obj.rbtree = rigidBodyTree;
            
            rotx = @(t) [1 0 0; 0 cos(t) -sin(t) ; 0 sin(t) cos(t)] ;
            roty = @(t) [cos(t) 0 sin(t) ; 0 1 0 ; -sin(t) 0  cos(t)] ;
            rotz = @(t) [cos(t) -sin(t) 0 ; sin(t) cos(t) 0 ; 0 0 1] ;
            
            obs_size=0.6;
            tmp=collisionBox(obs_size,obs_size,obs_size);
            obj.obstacles=tmp;
            T=eye(4);
            T(1:3,4)=[0.5,0.5,0.5];
            obj.obstacles(1).Pose=T;
            
            obj.obstacles(2)=collisionBox(obs_size,obs_size,obs_size);
            T=eye(4);
            T(1:3,4)=[-0.5,0.5,0.5];
            obj.obstacles(2).Pose=T;
            
            %show(col_obstacle)
            %hold on
            
            jnt1_len=1;
            jnt1_siz=0.1;
            jnt2_len=1;
            jnt2_siz=0.1;
            
            
            
            bodybase = rigidBody('base_link');
            jnt0 = rigidBodyJoint('based_fixed','fixed');
            T=eye(4,4);
            T(1:3,4)=offset;
            jnt0.setFixedTransform(T);
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
            tmp=collisionBox(jnt1_len,jnt1_siz,jnt1_siz);
            obj.cols=tmp;
            obj.col_names={body1_col.Name};
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
            obj.cols(2)=collisionBox(jnt2_len,jnt2_siz,jnt2_siz);
            obj.col_names{2}=body2_col.Name;
            
            tool = rigidBody('tool');
            jnt2_tool = rigidBodyJoint('joint_2_tool','fixed');
            T=eye(4,4);
            T(1,4)=jnt2_len;
            T(3,4)=0.5*(jnt1_siz+jnt2_siz);
            jnt2_tool.setFixedTransform(T);
            tool.Joint = jnt2_tool;
            addBody(obj.rbtree,tool,body2.Name);
            
            obj.rbtree.DataFormat='column';
            
            obj.conf=obj.rbtree.homeConfiguration;
            
            for ic=1:length(obj.cols)
                obj.cols(ic).Pose=obj.rbtree.getTransform(obj.conf,obj.col_names{ic});
            end
            
            
        end
        
        function r = check(obj,configuration)
            
            
            if length(configuration)~=3
                error('this example works only in 3d');
            end
           
            if or(sum(configuration<obj.lb),sum(configuration>obj.ub))
                r=0;
                return;
            end
            
            for ic=1:length(obj.cols)
                obj.cols(ic).Pose=obj.rbtree.getTransform(configuration,obj.col_names{ic});
            end
            
            r=1;
            for ic=1:length(obj.cols)
                for io=1:length(obj.obstacles)
                    if (checkCollision(obj.cols(ic),obj.obstacles(io)))
                        r=0;
                        return
                    end
                end
            end
        end
        
        function plot3d(obj,configuration)
            
            ish=ishold(gca);
            if length(configuration)~=3
                error('this example works only in 3d');
            end
          
            
            for ic=1:length(obj.cols)
                obj.cols(ic).Pose=obj.rbtree.getTransform(configuration,obj.col_names{ic});
            end
            
            
            
            for ic=1:length(obj.cols)
                show(obj.cols(ic))
                hold on
            end
            for io=1:length(obj.obstacles)
                show(obj.obstacles(io))
            end
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
