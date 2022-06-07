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
vmax = sqrt(1/c * b / (1 + g(3) * gamma) * umax);

% maximum accelerationg 3.2152 m/s^2
a_acc_max = 1/m * b/(1 + g(1) * gamma) * umax - 0; 
% maximum deacceleration
% for state 1 (-3.4528 m/s^2), however, it is a limit number and cannot actually be achieved
a_dec_max_1 =  1/m * b/(1 + g(1) * gamma) * umin - 1/m * c * v12^2;
% for state 2 (-2.7625 m/s^2), however, it is a limit number and cannot actually be achieved 
a_dec_max_2 =  1/m * b/(1 + g(2) * gamma) * umin - 1/m * c * v23^2;
% for state 3 (-3.5368 m/s^2), however, it is a limit number and cannot actually be achieved 
a_dec_max_3 =  1/m * b/(1 + g(3) * gamma) * umin - 1/m * c * vmax^2;
% maixmum deacceleartion is -3.3310 m/s^2
a_dec_max = min([a_dec_max_1, a_dec_max_2, a_dec_max_3]);

clear a_dec_max_1 a_dec_max_2 a_dec_max_3
%% step 2.2
% model with two-point format 
% by using maple, optimal alpha, beta are: alpha=28.8575, beta=249.8266
alpha = 28.8575;
beta = 249.8266;

%% step 2.3

test_t = 5;
step_3.y0 = [0;10];

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

[v, u] = Simulator_2_8(Np, Nc, lambda, [umin, umax], [vmin, vmax], a_comf_max,... 
                x_0, v_0, u_0, v_ref, Ts, [T_0, T_end], model, @(t,y) dydt_step8(t, y, m, gamma, b, c, g));

%% step 2.9

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

[v, u] = Simulator_2_8(Np, Nc, lambda, [umin, umax], [vmin, vmax], a_comf_max,... 
                x_0, v_0, u_0, v_ref, Ts, [T_0, T_end], model, @(t,y) dydt_step8(t, y, m, gamma, b, c, g));

            
%% step 2.10