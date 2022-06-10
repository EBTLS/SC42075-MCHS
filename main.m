%%%%%
%%% SC42075 Modelling and Control of Hybrid Systems
%%% Assignment
%%% Author: Jiaxuan Zhang, Yiting Li
%%%%%

clear 
close all
clc

%% Global Parameters
m = 800;
c = 0.4;
b = 3700;
umax = 1.3;
umin = -1.3;
vmin = 0;
a_comf_max = 2.5;
gamma = 0.87;
v12 = 15;
v23 = 30;

g = [1,2,3];

%% Step 2.1
% maximum speed: 57.7150 m/s;
vmax_1 = sqrt(1/c * b / (1 + g(1) * gamma) * umax);
vmax_2 = sqrt(1/c * b / (1 + g(2) * gamma) * umax);
vmax_3 = sqrt(1/c * b / (1 + g(3) * gamma) * umax);
vmax = min([vmax_1, vmax_2, vmax_3])

% maximum accelerationg 3.2152 m/s^2
a_acc_max = 1/m * b/(1 + g(1) * gamma) * umax - 0; 
% maximum deacceleration
% for state 1 (-3.3277 m/s^2), however, it is a limit number and cannot actually be achieved
a_dec_max_1 =  1/m * b/(1 + g(1) * gamma) * umin - 1/m * c * v12^2;
% for state 2 (-2.6443 m/s^2), however, it is a limit number and cannot actually be achieved 
a_dec_max_2 =  1/m * b/(1 + g(2) * gamma) * umin - 1/m * c * v23^2;
% for state 3 (-3.3310 m/s^2), however, it is a limit number and cannot actually be achieved 
a_dec_max_3 =  1/m * b/(1 + g(3) * gamma) * umin - 1/m * c * vmax^2;
% maixmum deacceleartion is -3.3310 m/s^2
a_dec_max = min([a_dec_max_1, a_dec_max_2, a_dec_max_3]);

% clear a_dec_max_1 a_dec_max_2 a_dec_max_3
%% step 2.2
% model with two-point format 
% by using maple, optimal alpha, beta are: alpha=28.8575, beta=249.8266
alpha = 28.8575;
beta = 249.8266;

figure;

v = [0: 0.1: vmax];
v1 = [0: 0.1: alpha];
v2 = [alpha: 0.1: vmax];
plot(v, c*v.^2, 'b');
hold on
plot(v1, beta/alpha*v1, 'r')
hold on
plot(v2, ((c*vmax^2 - beta)/(vmax-alpha)*(v2 - vmax) + c*vmax^2), 'r')
grid on
legend('V(v)', 'P(v)');
xlabel('v');
ylabel('m*F_{firction}');
title("approximation result")  

clear v v1 v2 
%% step 2.3

test_t = 5;
step_3.y0 = [0;44];

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



%% step 2.4

% See in .mw file

% for driving force part
% f1 = @(v,u) 1/m * b/(1 + gamma) * u;
% f2 = @(v,u) 1/m * b/(1 + 2 * gamma) * u;
% f3 = @(v,u) 1/m * b/(1 + 3 * gamma) * u;
% 
% % for friction part
% g1 = @(v) beta/alpha * v;
% g2 = @(v) 1/m * (c * vmax^2 - beta)/(vmax - alpha) * (v - vmax) + c * vmax^2;
% 
% f11 = f1 + 0.15 * f1


%% step 2.6

model = MLD_Model_3delta();


%% step 2.7
lambda = 0.1;
Np = 2;
Nc = 2;
x_0 = 5;
v_0 = [0];
u_0 = 0;
Ts = 0.15;
v_ref = [10; 10];

[flag, v, u, xc, uc] = Solution_2_7(Np, Nc, lambda, umax, umin, vmax, vmin, a_comf_max,... 
                v_0, u_0, model, Ts, v_ref);
            
            
%% step 2.8
% lambda = 0.1;
% Np = 2;
% Nc = 2;
% x_0 = 5;
% v_0 = [0];
% u_0 = 0;
% Ts = 0.15;
T_0 = 0;
T_end = 25;
v_ref = 5 * ones(length(T_0: Ts: T_end), 1);

[v, u, Result_constant_ref] = Simulator_2_8(Np, Nc, lambda, [umin, umax], [vmin, vmax], a_comf_max,... 
                x_0, v_0, u_0, v_ref, Ts, [T_0, T_end], model, @(t,y) dydt_step8(t, y, m, gamma, b, c, g));

%% step 2.9

lambda = 0.1;
Np = 5;
Nc = 4;
x_0 = 5;
v_0 = 0.9*alpha;
u_0 = 0;
Ts = 0.15;
T_0 = 0;
T_end = 25;
v_ref = GenerateXRef_2_8(Ts, alpha);

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
v_ref = GenerateXRef_2_8(Ts, alpha);

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

%% step 2.10
x0 = 0.9*alpha;
Np = 5;
Nc = 4;
v_ref = GenerateXRef_2_8(Ts, alpha)';
Nsim = length(v_ref);
temp = [];
for i = 1:Nsim
    temp = [temp, repmat(v_ref(i), 1, 1)];
end
xref = temp;

clear temp

[ctrl, sys] = Solution_2_10(Np, Nc, lambda, umax, umin, vmax, vmin, a_comf_max,... 
                v_0, u_0, model, Ts, v_ref, 1);
%%

loop_im = ClosedLoop(ctrl, sys);

data_im = loop_im.simulate(x0, Nsim, 'x.reference', xref, 'u.previous', u_0);

figure;

subplot(2, 1, 1); 
plot(1:Nsim, data_im.X(1:Nsim), 'linewidth', 2); 
title('im state');
hold on
plot(1:Nsim, v_ref(1:Nsim), 'r');
hold on
plot(1:Nsim, sys.x.min*ones(1,Nsim), 'k--', 1:Nsim, sys.x.max*ones(1,Nsim), 'k--');
axis([1 Nsim 0 vmax]);
grid on

subplot(2, 1, 2); 
stairs(1:Nsim, data_im.U, 'linewidth', 2); 
title('output');
hold on
plot(1:Nsim, sys.u.min*ones(1,Nsim), 'k--', 1:Nsim, sys.u.max*ones(1,Nsim), 'k--');
axis([1 Nsim -1.3 1.3]);
grid on
%%
Np = 5;
Nc = 4;

[explicit_ctrl_5_4, sys] = Solution_2_10(Np, Nc, lambda, umax, umin, vmax, vmin, a_comf_max,... 
                v_0, u_0, model, Ts, v_ref, 0);
%%
Np = 6;
Nc = 5;

[explicit_ctrl_9_8, sys] = Solution_2_10(Np, Nc, lambda, umax, umin, vmax, vmin, a_comf_max,... 
                v_0, u_0, model, Ts, v_ref, 0);
            
%%

loop_explicit_5_4 = ClosedLoop(explicit_ctrl_5_4, sys);
data_explicit_5_4 = loop_explicit_5_4.simulate(x0, Nsim, 'x.reference', xref, 'u.previous', u_0);
loop_explicit_9_8 = ClosedLoop(explicit_ctrl_9_8, sys);
data_explicit_9_8 = loop_explicit_9_8.simulate(x0, Nsim, 'x.reference', xref, 'u.previous', u_0);


figure;
Nsim = length(v_ref);
subplot(2, 1, 1); 
plot(1:Nsim, data_explicit_5_4.X(1:Nsim), 'b'); 
hold on
plot(1:Nsim, data_explicit_9_8.X(1:Nsim), 'r'); 
title('e state');
hold on
plot(1:Nsim, v_ref(1:Nsim));
hold on
plot(1:Nsim, sys.x.min*ones(1,Nsim), 'k--', 1:Nsim, sys.x.max*ones(1,Nsim), 'k--');
axis([1 Nsim 0 vmax]);
grid on

subplot(2, 1, 2); 
stairs(1:Nsim, data_explicit_5_4.U, 'b'); 
hold on
stairs(1:Nsim, data_explicit_9_8.U, 'r'); 
title('output');
hold on
plot(1:Nsim, sys.u.min*ones(1,Nsim), 'k--', 1:Nsim, sys.u.max*ones(1,Nsim), 'k--');
axis([1 Nsim -1.3 1.3]);
grid on