%%%%%
%%% SC42075 Modelling and Control of Hybrid Systems
%%% Assignment
%%% Author: Jiaxuan Zhang, Yiting Li
%%%%%

clear 
close all
clc

%% add path
addpath('./src')

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

Script_2_2

%% step 2.3

test_t = 5;
step_3.y0 = [0;44];

Script_2_3


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
T_0 = 0;
T_end = 25;
v_ref = 5 * ones(length(T_0: Ts: T_end), 1);

[v, u, Result_constant_ref] = Simulator_2_8(Np, Nc, lambda, [umin, umax], [vmin, vmax], a_comf_max,... 
                x_0, v_0, u_0, v_ref, Ts, [T_0, T_end], model, @(t,y) dydt_step8(t, y, m, gamma, b, c, g));

%% step 2.9

Script_2_9

%% step 2.10

Script_2_10

