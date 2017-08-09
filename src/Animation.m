clc;
clear;
x = csvread('trafficX.csv');
y = x(1,1:end-1)*0;

xlabel('Position(m)');
ylabel('Lane');
title('Traffic Flow');

path = animatedline('LineStyle','none','Marker','.');
set(gca,'XLim',[x(1,end-1) x(end,1)],'YLim',[-1 1]);
%hold on;

for i=1:size(x,1)
    clearpoints(path)
    addpoints(path,x(i,1:end-1),y);
    drawnow;
    pause(.01);
end
    
    
