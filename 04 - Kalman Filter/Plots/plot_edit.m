% Load old plot
open 09_Kalman_Control_Comp_06.fig

D = get(gca, 'Children');
XData = get(D, 'XData');
YData = get(D, 'YData');

% Create new plot

ts = 0.02;
t = 1:1:length(XData{1}(1,:));

figure(6)
hold on
title("Pitch")
xlabel('Time')
ylabel('Angle (rad)')
plot(t, YData{2}(1,:))
plot(t, YData{8}(1,:))
plot(t, YData{1}(1,:))

legend("IMU pitch", "Estimated pitch", "Encoder pitch")