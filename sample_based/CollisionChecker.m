classdef CollisionChecker < handle
    properties
        js_msg
        js_sub
        collision_check
        col_req
        fake
        fake_obj=[-0.5000   -3.1414    2   -0.5000    1.5708   -0.0001   -0.000]';;
        fake_obj_radius=0.3;
    end
    methods
        function r = init(obj,group_name,fake)
            if nargin<3
                fake=0;
            end
            if fake
                obj.fake=fake;
                r=1;
            else
                
                obj.js_sub = rossubscriber('/joint_states');
                try
                    obj.js_msg = receive(obj.js_sub,10);
                    r=1;
                    
                    obj.collision_check = rossvcclient('/check_state_validity');
                    obj.col_req = rosmessage(obj.collision_check);
                    
                    obj.col_req.GroupName=group_name;
                    
                catch
                    r=0;
                end
            end
        end
        function r = check(obj,position)
            if obj.fake
                r=norm(position-obj.fake_obj)>obj.fake_obj_radius;
                return
            end
            obj.js_msg.Position=position;
            obj.col_req.RobotState.JointState=obj.js_msg;
            
            col_res=obj.collision_check.call(obj.col_req);
            r=col_res.Valid;
        end
        function r = checkPath(obj,path)
            for idx=1:size(path,2)
                r=check(obj,path(:,idx));
                if (r==0)
                    break;
                end
            end
        end
        function pos=getActualPosition(obj)
            if (obj.fake)
                pos=zeros(length(obj.fake_obj),1);
                return;
            end
            obj.js_msg = receive(obj.js_sub,10);
            pos=obj.js_msg.Position;
        end
        function names=getNames(obj)
            if (obj.fake)
                names={'ur10_shoulder_pan_joint',
                    'ur10_shoulder_lift_joint',
                    'ur10_elbow_joint',
                    'ur10_wrist_1_joint',
                    'ur10_wrist_2_joint',
                    'ur10_wrist_3_joint', 
                    'linear_joint'};
                    return
            end
            names=obj.js_msg.Name;
        end
    end
end