function [pos,vel,total_duration]=bound_velocity_time_parametrization(path,max_vel,t)

nc=length(path.connections);

for ic=1:nc
    conn=path.connections(ic);
    dir(:,ic)=(conn.getChild.q-conn.getParent.q);
    duration(ic,1)=max(abs(dir(:,ic))./max_vel);
    velocity(:,ic)=dir(:,ic)/duration(ic,1);
    
end

time=[0;cumsum(duration)];
total_duration=time(end);
if t<0
    pos=path.connections(1).getParent.q;
    vel=0*max_vel;
elseif t>time(end)
    pos=path.connections(end).getChild.q;
    vel=0*max_vel;
else
    idx=find(time<=t,1,'last');
    vel=velocity(:,idx);
    pos=path.connections(idx).getParent.q+vel*(t-time(idx));
end