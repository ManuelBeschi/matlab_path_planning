classdef InformedSampler < handle
    %INFORMED_SAMPLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        lb
        ub
        goal_conf
        start_conf
        cost
        ellipse_center
        ellipse_max_radius
        ellipse_min_radius
        focii_distance
        ndof
        ellipse_axis
        rot_matrix;
    end
    
    methods
        function obj = InformedSampler(start_conf,goal_conf,lb,ub,cost)
            obj.lb=lb;
            obj.ub=ub;
            obj.ndof=length(lb);
            if isobject(start_conf)
                obj.start_conf=start_conf.q;
            else
                obj.start_conf = start_conf;
            end
            
            if isobject(start_conf)
                obj.goal_conf=goal_conf.q;
            else
                obj.goal_conf=goal_conf;
            end
            
            obj.ellipse_center=(obj.goal_conf+obj.start_conf)*0.5;
            obj.focii_distance=norm(obj.goal_conf-obj.start_conf);
            obj.rot_matrix=computeRotationaMatrix(obj.goal_conf-obj.start_conf);
            if nargin>4
                obj.setCost(cost);
            else
                obj.setCost(inf);
            end
        end
        
        function q = sample(obj)
            if (obj.cost<inf)
               flag=true;
               while flag
                   r=nthroot(rand,length(obj.lb));
                   sphere=rand(length(obj.lb),1);
                   unit_sphere=r*sphere/norm(sphere);
                   q=obj.ellipse_center+obj.ellipse_axis.*unit_sphere;
                   flag=~all((obj.lb<=q).*(q<=obj.ub));
               end
            else
                q=obj.lb+(obj.ub-obj.lb).*rand(length(obj.lb),1);
            end
        end
        
        function setCost(obj,cost)
            obj.cost=cost;
            if (cost<obj.focii_distance)
                warning('cost is smaller than focii distance');
                obj.ellipse_max_radius=0.5*obj.focii_distance;
                obj.ellipse_min_radius=0;
            else
                obj.ellipse_max_radius=0.5*cost;
                obj.ellipse_min_radius=0.5*sqrt(cost^2-obj.focii_distance^2);
            end
            obj.ellipse_axis=obj.ellipse_min_radius*ones(obj.ndof,1);
            obj.ellipse_axis(1)=obj.ellipse_max_radius;
        end
        
    end
end

