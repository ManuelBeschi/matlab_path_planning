function [p1,p2] = plotPercentile(x,y,n_fig,color)

percentile = 95;

mx = mean(x);
my = mean(y);

px_h = prctile(x,percentile);
px_l = prctile(x,100-percentile);

py_h = prctile(y,percentile);
py_l = prctile(y,100-percentile);

figure(n_fig)
% plot(mx,my,'o','Color',color);
plot(mx,my,'o','Color',color,'MarkerSize',5,'LineWidth',2,'MarkerFaceColor',color);
p1=plot([px_l px_h],[my my],'-','Color',color,'LineWidth',2);
p2=plot([mx mx],[py_l py_h],'Color',color,'LineWidth',2);


end

