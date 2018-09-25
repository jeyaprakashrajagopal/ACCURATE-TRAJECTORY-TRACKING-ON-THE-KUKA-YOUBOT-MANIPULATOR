%graphics_toolkit ("gnuplot");

%figure;
yyaxis left;
data = dlmread("tofindratio.txt", ",");
t1 = 8500;
a = size(data(1:t1, 4));

hold all;
    
t = title('Different torques for joint 4');
set(t, 'FontSize', 24);
xl = xlabel ('time(seconds)'); 
set(xl, 'FontSize', 20);
yl = ylabel ('Torque (N.m.)');
set(yl, 'FontSize', 20);

plot(data(1:t1, 4), data(1:t1, 2), 'Color', 'b');
plot(data(1:t1, 4), data(1:t1, 3), 'Color', 'r');

yyaxis right;
y2l = ylabel('\color{black} q (rad)');
set(y2l, 'FontSize', 20);
plot(data(1:t1, 4), data(1:t1, 1), 'Color', 'k');

set(findall(gca, 'Type', 'Line'),'LineWidth',3)
legend('Controller', 'Gravity', 'q', 'Location', 'north');

print -dpng joint4_ratio.png;
