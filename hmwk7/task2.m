close all;
clear;

distance = [41.1716,43.8966,37.2594,43.4215,36.8929,37.9595,35.7629,38.3734,31.743,33.5749,34.0959,34.4657,29.5435,31.7048,26.1274,31.708,22.4097];
% distance = [41.735,44.6964,37.0125,41.997,34.517,38.8371,34.1898,36.4787,31.5758,32.331,31.1395,31.9571,28.0487,28.2873,26.8464,27.4564,23.9823];
% distance = [44.2376,46.9524,39.1015,44.3418,37.0119,40.5749,38.5246,38.1082,31.596,34.9731,31.8693,34.5226,28.1356,28.8966,28.6267,28.6069,25.4737];
distance = distance * 15.25;

frame = 1:1:length(distance);

scatter(frame,distance);

F = polyfit(frame, distance, 1);

b = F(2);
m = F(1);
Y0 = -b/m

hold on;

plot([0, Y0], [b, 0]);
title('time to impact');
xlabel('frame number');
ylabel('estimated distance to impact (mm)');