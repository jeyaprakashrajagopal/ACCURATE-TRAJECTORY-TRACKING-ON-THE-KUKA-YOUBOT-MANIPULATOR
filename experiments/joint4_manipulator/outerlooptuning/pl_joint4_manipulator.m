%graphics_toolkit ("gnuplot");

figure;

data = dlmread("manipulator_joint4_outerloop_0pt5rad.txt", ",");
a = size(data(:, 1));

setPoint = 2.23184;

hold all;
%  Add a horizontal line for the Temperature at steady state
line('XData', [data(a(:, 1), 2) 0], 'YData', [setPoint setPoint], 'LineStyle', '-', ...
    'LineWidth', 2, 'Color','m');
    
t = title('Joint 4 empirical tuning');
set(t, 'FontSize', 24); 
xl = xlabel ('time(seconds)');
set(xl, 'FontSize', 20);
yl = ylabel ('Position (rad)');
set(yl, 'FontSize', 20);

%plot(data(:, 3), data(:, 1), 'Color', 'b', data(:, 3), data(:, 2), '--', 'Color', 'r');
plot(data(:, 2), data(:, 1));
set(findall(gca, 'Type', 'Line'),'LineWidth',5);
legend('Setpoint', 'Measured values', 'Location', 'South');
print -dpng joint4_outerloop_0pt5rad.png;
