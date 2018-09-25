%graphics_toolkit ("gnuplot");

figure;

data = dlmread("manipulator_joint5.txt", ",");
t1 = 8700;
a = size(data(:, 2));
setPoint = 2.9106;

hold all;
% Add a horizontal line for the Temperature at steady state
line('XData', [data(a(1, 1), 3) 0], 'YData', [setPoint setPoint], 'LineStyle', '-', ...
    'LineWidth', 1, 'Color','m');

t = title('Trajectory tracking on joint 5');
set(t, 'FontSize', 24);
xl = xlabel ('time(seconds)'); 
set(xl, 'FontSize', 20);
yl = ylabel ('Position (rad)');
set(yl, 'FontSize', 20);

plot(data(:, 3), data(:, 1), 'Color', 'b');
plot(data(:, 3), data(:, 2), '--', 'Color', 'r');

%plot(data(:, 2), data(:, 1), '--', 'Color', 'r');
set(findall(gca, 'Type', 'Line'),'LineWidth',3)
legend('Initial position', 'Expected', 'Measured', 'Location', 'southwest');

print -dpng joint5_trajectory.png;
