test_t = 5;
step_3.y0 = [0;22];

% original function simulation
[temp_t, temp_y] = ode45(@(t,y) dydt_step3(t,y,0,alpha,beta,m,gamma,b,c,vmax),...
    [0,test_t],step_3.y0);
step_3.original_simulation.t = temp_t;
step_3.original_simulation.y = temp_y;

% PWA function simulation
[temp_t, temp_y] = ode45(@(t,y) dydt_step3(t,y,1,alpha,beta,m,gamma,b,c,vmax),...
    [0,test_t],step_3.y0);
step_3.pwa_simulation.t = temp_t;
step_3.pwa_simulation.y = temp_y;

% plot the result
figure
% position result
subplot(1, 2, 1)
plot(step_3.original_simulation.t, step_3.original_simulation.y(:,1), 'r');
hold on
plot(step_3.pwa_simulation.t, step_3.pwa_simulation.y(:,1), 'b');
grid on
legend('original function', 'pwa function')
xlabel('t')
ylabel('position')
title("step 3 simulation position")

% speed result
subplot(1, 2, 2)
plot(step_3.original_simulation.t, step_3.original_simulation.y(:,2), 'r');
hold on
plot(step_3.pwa_simulation.t, step_3.pwa_simulation.y(:,2), 'b');
grid on
legend('original function', 'pwa function')
xlabel('t')
ylabel('speed')
title("step 3 simulation speed")