lambda = 0.1;
Np = 5;
Nc = 4;
x_0 = 5;
v_0 = 0.9*alpha;
u_0 = 0;
Ts = 0.15;
T_0 = 0;
T_end = 25;
v_ref = GenerateXRef_2_9(Ts, alpha);

[v, u, Results_varying_ref_5_4] = Simulator_2_8(Np, Nc, lambda, [umin, umax], [vmin, vmax], a_comf_max,... 
                x_0, v_0, u_0, v_ref, Ts, [T_0, T_end], model, @(t,y) dydt_step8(t, y, m, gamma, b, c, g));

lambda = 0.1;
Np = 9;
Nc = 8;
x_0 = 5;
v_0 = 0.9*alpha;
u_0 = 0;
Ts = 0.15;
T_0 = 0;
T_end = 25;
v_ref = GenerateXRef_2_9(Ts, alpha);

[v, u, Results_varying_ref_9_8] = Simulator_2_8(Np, Nc, lambda, [umin, umax], [vmin, vmax], a_comf_max,... 
                x_0, v_0, u_0, v_ref, Ts, [T_0, T_end], model, @(t,y) dydt_step8(t, y, m, gamma, b, c, g));     

            
figure
plot(Results_varying_ref_9_8.x_history, Results_varying_ref_9_8.v_history, 'r');
hold on;
plot(Results_varying_ref_5_4.x_history, Results_varying_ref_5_4.v_history, 'b');
grid on;
legend('N_p = 9, N_c = 8', 'N_p = 5, N_c = 4');
xlabel('x');
ylabel('v');
title("simulation result: trajectories of (x,v)")

figure
plot([T_0: Ts: T_end], Results_varying_ref_9_8.x_history, 'r');
hold on;
plot([T_0: Ts: T_end], Results_varying_ref_5_4.x_history, 'b');
grid on;
legend('N_p = 9, N_c = 8', 'N_p = 5, N_c = 4');
xlabel('t');
ylabel('x');
title("simulation result: x")

figure
plot([T_0: Ts: T_end], Results_varying_ref_9_8.v_history, 'r');
hold on;
plot([T_0: Ts: T_end], Results_varying_ref_5_4.v_history, 'b');
hold on;
plot([T_0: Ts: T_end], v_ref);
grid on;
legend('N_p = 9, N_c = 8', 'N_p = 5, N_c = 4', 'v_{ref}');
xlabel('t');
ylabel('v');
title("simulation result: v")

            
figure
plot([T_0: Ts: T_end], Results_varying_ref_9_8.v_diff, 'r');
hold on;
plot([T_0: Ts: T_end], Results_varying_ref_5_4.v_diff, 'b');
grid on;
legend('N_p = 9, N_c = 8', 'N_p = 5, N_c = 4');
xlabel('t');
ylabel('(v - v_{ref})');
title("simulation result: (v - v_{ref})")

figure
plot([T_0: Ts: T_end], Results_varying_ref_9_8.u_history, 'r');
hold on;
plot([T_0: Ts: T_end], Results_varying_ref_5_4.u_history, 'b');
grid on;
legend('N_p = 9, N_c = 8', 'N_p = 5, N_c = 4');
xlabel('t');
ylabel('u');
title("simulation result: u")            
figure
plot([T_0: Ts: T_end], Results_varying_ref_9_8.u_diff, 'r');
hold on
plot([T_0: Ts: T_end], Results_varying_ref_5_4.u_diff, 'b');
grid on;
legend('N_p = 9, N_c = 8', 'N_p = 5, N_c = 4');
xlabel('t');
ylabel('\Delta u');
title("simulation result: \Delta u")