clc;
close all;

leftPositions = load('leftPositions.txt');
rightPositions = load('rightPositions.txt');

x = leftPositions(:,1) -6.5;
y = -(leftPositions(:,2)-10);
z = -leftPositions(:,3);

ZYcoefs = polyfit(z,y,2);
ZXcoefs = polyfit(z,x,1);

fzy = @(v) ZYcoefs(1).*v.^2 + ZYcoefs(2).*v + ZYcoefs(3);

fzx = @(v) ZXcoefs(1).*v + ZXcoefs(2);

zinterval = [-450 0];

figure();

plot(z,y,'s');
title('Ball Position');
xlabel('z location (inch)');
ylabel('y location (inch)');

hold on;

fplot(@(v) ZYcoefs(1).*v.^2 + ZYcoefs(2).*v + ZYcoefs(3),zinterval);

legend('ball location','prediction line');

figure();
plot(z,x,'s');
title('Ball Position');
xlabel('z location (inch)');
ylabel('x location (inch)');

hold on; 

fplot(@(v) ZXcoefs(1).*v + ZXcoefs(2),zinterval);

legend('ball location','prediction line');


