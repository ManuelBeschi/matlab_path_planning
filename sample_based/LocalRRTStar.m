classdef LocalRRTStar < RRTStar
    %RRTSTAR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        local_sampler
    end
    
    methods
        function obj = LocalRRTStar(tree,goal_node,sampler,r_rewire)
            obj=obj@RRTStar(tree,goal_node,sampler,r_rewire);
            obj.local_sampler=LocalInformedSampler(sampler.start_conf,sampler.goal_conf,sampler.lb,sampler.ub,sampler.cost);
        end
        
        function improved=step(obj,checker)
            conn=obj.tree.getConnectionToNode(obj.goal_node);
            obj.local_sampler.setCost(obj.tree.costToNode(obj.goal_node));
            improved=false;
            for idx=1:length(conn)
                nodes(:,idx)=conn(idx).getParent.q;
            end
            nodes(:,length(conn)+1)=conn(idx).getChild.q;
            
            for idx=2:(length(nodes)-1)
                radius=0.5*norm(nodes(:,idx-1)-nodes(:,idx+1));
                obj.local_sampler.setBall(nodes(:,idx),radius);
                improved_this_step=obj.tree.rewire(obj.sampler.sample,checker);
                
                improved=improved || improved_this_step;
            end
        end
        
    end
end

