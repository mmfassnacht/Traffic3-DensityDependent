clc;
clear;
% x1 = csvread('trafficX1.csv');
%  x2 = csvread('trafficX2.csv');
 x3 = csvread('trafficX3.csv');
%  x4 = csvread('trafficX4.csv');
% x5 = csvread('trafficX5.csv');

% y1 = x1(1,1:end-1)*0 + 80;
%  y2 = x2(1,1:end-1)*0 + 90;
 y3 = x3(1,1:end-1)*0 + 100;
%  y4 = x1(1,1:end-1)*0 + 4;
% % y5 = x1(1,1:end-1)*0 + 5;

xlabel('Position(m)');
ylabel('Initial Distance Between Cars (m)');
title('Sudden Stop From 30 m/s');

path = animatedline('LineStyle','none','Marker','.');
set(gca,'XLim',[x3(1,end-1) x3(end,1)+50],'YLim',[90 110]);

for i=1:size(x3,1)
    clearpoints(path)
%     set(gca,'XLim',[x1(i,end-1)-1 x1(i,1)+1],'YLim',[70 110]);
%     addpoints(path,x1(i,1:end-1),y1);
%      addpoints(path,x2(i,1:end-1),y2);
     addpoints(path,x3(i,1:end-1),y3);
%     addpoints(path,x4(i,1:end-1),y4);
% %     addpoints(path,x5(i,1:end-1),y5);
    drawnow; 
    F(i) = getframe(gcf);
    pause(.01);   
end
   set(gca,'XLim',[x3(end,end-1)-1 x3(end,1)+2],'YLim',[90 110]);
  F(size(x3,1)+1) = getframe(gcf);
  F(size(x3,1)+2) = getframe(gcf);
  F(size(x3,1)+3) = getframe(gcf);

video = VideoWriter('Traffic.avi','MPEG-4');
video.FrameRate = 30;
open(video);
writeVideo(video,F);
close(video);

    
