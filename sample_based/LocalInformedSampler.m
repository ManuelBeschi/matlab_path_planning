classdef LocalInformedSampler < InformedSampler
    %INFORMED_SAMPLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        configuration
        radius
    end
    
    methods
        function obj = LocalInformedSampler(start_conf,goal_conf,lb,ub,cost)
            
            obj=obj@InformedSampler(start_conf,goal_conf,lb,ub,cost);
            obj.configuration=(start_conf+goal_conf)*0.5;
            obj.radius=norm(start_conf-goal_conf)*2;
        end
        
        function setBall(obj,configuration,radius)
            obj.configuration=configuration;
            obj.radius=radius;
        end
        
        function q = sample(obj)
            
            flag=true;
            while flag
                r=nthroot(rand,length(obj.lb));
                sphere=randn(length(obj.lb),1);
                unit_sphere=r*sphere/norm(sphere);
                q=obj.configuration+obj.radius*unit_sphere;
                
                flag=~all((obj.lb<=q).*(q<=obj.ub));
                if (obj.cost<inf)
                    x=obj.rot_matrix'*(q-obj.ellipse_center)./obj.ellipse_axis;
                    flag= norm(x)>1;
                end
            end
            
        end
        
        function s=plotEllipsoid(obj)
            s=plotEllipsoid@InformedSampler(obj);
            if (obj.ndof>3)
                error('plot ellipsoid is available only with 3 or less joints')
            end
            if (obj.ndof==3)
                hold on
                [x,y,z] = ellipsoid(obj.configuration(1),obj.configuration(2),obj.configuration(3),...
                    obj.radius,obj.radius,obj.radius);

                s=[s surf(x,y,z,'FaceAlpha',0.5)];
            end
            
        end 
        
    end
end

