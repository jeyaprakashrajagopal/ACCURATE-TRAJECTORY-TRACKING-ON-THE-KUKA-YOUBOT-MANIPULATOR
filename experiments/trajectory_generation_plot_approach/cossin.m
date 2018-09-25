yyaxis left;
t = title('Joint 5 trajectory generation');
set(t, 'FontSize', 24);
xl = xlabel ('time(seconds)'); 
set(xl, 'FontSize', 20);
yl = ylabel ({'\color{blue}q (rad)'});
set(yl, 'FontSize', 20);

q_init = 2.9106;
JntLimit = 5.3469;

t = 0:0.001:10;
T = 10;

hold all;
y =  (JntLimit-q_init) * sin(2*pi/T*t) + q_init;

a = size(t);

% Add a horizontal line for the Temperature at steady state
line('XData', [0 10], 'YData', [q_init q_init], 'LineStyle', '-', ...
    'LineWidth', 1, 'Color','m');
plot(t,y, 'Color', 'b');

ydot = (JntLimit-q_init) * (2 * pi / T) * cos(2*pi/T*t);

yyaxis right;

y2l = ylabel('\color{red} qd (rad/s)');
set(y2l, 'FontSize', 20);

plot(t, ydot, '--', 'Color', 'r');

set(findall(gca, 'Type', 'Line'),'LineWidth', 4, 'defaulttextinterpreter','latex');
legend('Initial position', 'q', 'qd','Location','southwest');

print -dpng sinewave.png;
