classdef CollisionChecker < handle
    properties
        js_msg
        js_sub
        collision_check
        col_req
    end
    methods
        function r = init(obj,group_name)
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
        function r = check(obj,position)
            obj.js_msg.Position=position;
            obj.col_req.RobotState.JointState=obj.js_msg;
            
            col_res=obj.collision_check.call(obj.col_req);
            r=col_res.Valid;
        end
        function r = checkPath(obj,path)
            for idx=1:size(path,2)
                obj.js_msg.Position=path(:,idx);
                obj.col_req.RobotState.JointState=obj.js_msg;
                
                col_res=obj.collision_check.call(obj.col_req);
                r=col_res.Valid;
                if (r==0)
                    break;
                end
            end
        end
        function pos=getActualPosition(obj)
            obj.js_msg = receive(obj.js_sub,10);
            pos=obj.js_msg.Position;
        end
        function names=getNames(obj)
            names=obj.js_msg.Name
        end
    end
end