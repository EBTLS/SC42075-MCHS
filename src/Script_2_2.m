figure;

v = [0: 0.1: vmax];
v1 = [0: 0.1: alpha];
v2 = [alpha: 0.1: vmax];
plot(v, c*v.^2, 'r');
hold on
plot(v1, beta/alpha*v1, 'b')
hold on
plot(v2, ((c*vmax^2 - beta)/(vmax-alpha)*(v2 - vmax) + c*vmax^2), 'b')
grid on
legend('V(v)', 'P(v)');
xlabel('v');
ylabel('m*F_{firction}');
title("approximation result")  

clear v v1 v2 