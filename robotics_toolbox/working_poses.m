clear all;close all;clc;

% questo file genera delle configurazioni del robot per raggiungere dei
% punti nello spazio con la base del robot posizionata in diversi punti
% della cella. 
% NON SERVE PER L'OTTIMIZZAZIONE

load poses.mat % T_world_tool posizioni da raggiungere nello spazio cartesiano
% checker -> collision checker

% lower and upper bound
lb=-pi*ones(3,1);
ub=pi*ones(3,1);
metrics=Metrics;
weights = [0,0,0,1,1,1];
guess=[0;0;0];


tests=[];

while length(tests)<10
    offset=-0.5+rand(3,1);
    
    
    checker=Anthropomorphic3d;
    checker.init(lb,ub,offset);
    robot=checker.rbtree;
    ik = inverseKinematics('RigidBodyTree',robot);
    %%
    
    offset_ok=1;
    for idx=1:size(T_world_tool,3)
        
        ik_ok=0;
        for itrial=1:10
            if itrial>1
                guess=-pi+2*pi*rand(3,1);
            else
                [configSoln,solnInfo] = ik('tool',T_world_tool(:,:,idx),weights,guess);
            end
            
            if and(solnInfo.ExitFlag==1,checker.check(configSoln))
                ik_ok=1;
                break
            end
        end
       
        if (~ik_ok)
            offset_ok=0;
            break;
        end
        configurations(:,idx)=configSoln;

    end
    if (offset_ok==0)
        continue;
    end
    test.offset=offset;
    test.configurations=configurations;
    tests=[tests;test];
    
end