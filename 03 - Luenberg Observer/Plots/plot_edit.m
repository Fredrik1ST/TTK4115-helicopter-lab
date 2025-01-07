% Load old plot
open fig09_results_p2.fig

D = get(gca, 'Children');
XData = get(D, 'XData');
YData = get(D, 'YData');


% Create new plot

ts = 0.02;
t = 1:1:length(XData{1}(1,:));

figure(1)
hold on
title("Pitch")
xlabel('Time')
ylabel('Angle (rad)')
plot(t, YData{2}(1,:))
plot(t, YData{1}(1,:))
plot(t, YData{3}(1,:))

legend("IMU", "Encoder", "Observer")