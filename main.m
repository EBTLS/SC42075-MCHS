%%%%%
%%% SC42075 Modelling and Control of Hybrid Systems
%%% Assignment
%%% Author: Jiaxuan Zhang, Yiting Li
%%%%%

%% Global Parameters
m = 800;
c = 0.4;
b = 3700;
umax = 1.3;
umin = -1.3;
a_comf_max = 2.5;
gamma = 0.8;
v12 = 15;
v23 = 30;

g = [1,2,3];

%% Step 2.1
% maximum speed: 59.4707 m/s;
vmax = sqrt(1/c * b / (1 + g(3) * gamma) * umax);

% maximum accelerationg 3.3403 m/s^2
a_acc_max = 1/m * b/(1 + g(1) * gamma) * umax - 0; 
% maximum deacceleration
% for state 1 (-3.4528 m/s^2), however, it is a limit number and cannot actually be achieved
a_dec_max_1 =  1/m * b/(1 + g(1) * gamma) * umin - 1/m * c * v12^2;
% for state 2 (-2.7625 m/s^2), however, it is a limit number and cannot actually be achieved 
a_dec_max_2 =  1/m * b/(1 + g(2) * gamma) * umin - 1/m * c * v23^2;
% for state 3 (-3.5368 m/s^2), however, it is a limit number and cannot actually be achieved 
a_dec_max_3 =  1/m * b/(1 + g(3) * gamma) * umin - 1/m * c * vmax^2;
% maixmum deacceleartion is -3.5368 m/s^2
a_dec_max = min([a_dec_max_1, a_dec_max_2, a_dec_max_3]);

%% step 2.2

