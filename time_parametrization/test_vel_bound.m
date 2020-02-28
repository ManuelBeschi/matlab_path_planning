%clear all;close all;clc
max_vel=ones(3,1);

[pos,vel,total_duration]=bound_velocity_time_parametrization(path,max_vel,0);
t=(0:1e-2:total_duration)';
for idx=1:length(t)
    [pos(:,idx),vel(:,idx)]=bound_velocity_time_parametrization(path,max_vel,t(idx));
end

subplot(121)
plot(t,pos)
subplot(122)
plot(t,vel)