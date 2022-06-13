x0 = 0.9*alpha;
v_ref = GenerateXRef_2_9(Ts, alpha)';
Nsim = length(v_ref);
temp = [];
for i = 1:Nsim
    temp = [temp, repmat(v_ref(i), 1, 1)];
end
xref = temp;

clear temp

%% Failed Attemps: Manually Explicit Controller: Generation Too Slow

% lambda = 0.1;
% Np = 2;
% Nc = 2;
% x_0 = 5;
% v_0 = [0];
% u_0 = 0;
% Ts = 0.15;
% 
% [explicit_ctrl, sys] = Solution_2_10(Np, Nc, lambda, umax, umin, vmax, vmin, a_comf_max,... 
%                         model, Ts, 2);


%% Generate Controllers

Np = 2;
Nc = 2;

[ctrl_2_2, sys] = Solution_2_10(Np, Nc, lambda, umax, umin, vmax, vmin, a_comf_max,... 
                model, Ts, 1);
[explicit_ctrl_2_2, sys] = Solution_2_10(Np, Nc, lambda, umax, umin, vmax, vmin, a_comf_max,... 
                model, Ts, 0);

Np = 3;
Nc = 3;            
[ctrl_3_3, sys] = Solution_2_10(Np, Nc, lambda, umax, umin, vmax, vmin, a_comf_max,... 
                model, Ts, 1);
[explicit_ctrl_3_3, sys] = Solution_2_10(Np, Nc, lambda, umax, umin, vmax, vmin, a_comf_max,... 
                model, Ts, 0);

Np = 4;
Nc = 4;
[ctrl_4_4, sys] = Solution_2_10(Np, Nc, lambda, umax, umin, vmax, vmin, a_comf_max,... 
                model, Ts, 1);            
[explicit_ctrl_4_4, sys] = Solution_2_10(Np, Nc, lambda, umax, umin, vmax, vmin, a_comf_max,... 
                model, Ts, 0);

%% simulation: implicit controller
t1 = tic;               
% loop_im_2_2 = ClosedLoop(ctrl_2_2, sys);
% data_im_2_2 = loop_im_2_2.simulate(x0, Nsim, 'x.reference', xref, 'u.previous', u_0);
v_ref = GenerateXRef_2_9(Ts, alpha);
[v, u, Results_2_2] = Simulator_2_8(2, 2, lambda, [umin, umax], [vmin, vmax], a_comf_max,... 
                x_0, v_0, u_0, v_ref, Ts, [T_0, T_end], model, @(t,y) dydt_step8(t, y, m, gamma, b, c, g));     

T1 = toc(t1);
t2 = tic ;
% loop_im_3_3 = ClosedLoop(ctrl_3_3, sys);
% data_im_3_3 = loop_im_3_3.simulate(x0, Nsim, 'x.reference', xref, 'u.previous', u_0);
[v, u, Results_3_3] = Simulator_2_8(3, 3, lambda, [umin, umax], [vmin, vmax], a_comf_max,... 
                x_0, v_0, u_0, v_ref, Ts, [T_0, T_end], model, @(t,y) dydt_step8(t, y, m, gamma, b, c, g));     

T2 = toc(t2);
t3 = tic;
% loop_im_4_4 = ClosedLoop(ctrl_4_4, sys);
% data_im_4_4 = loop_im_4_4.simulate(x0, Nsim, 'x.reference', xref, 'u.previous', u_0);
[v, u, Results_4_4] = Simulator_2_8(4, 4, lambda, [umin, umax], [vmin, vmax], a_comf_max,... 
                x_0, v_0, u_0, v_ref, Ts, [T_0, T_end], model, @(t,y) dydt_step8(t, y, m, gamma, b, c, g));     

T3 = toc(t3);

%% simulation: explicit controller

% x0 = 0.9*alpha;
% u0 = 0; 
% x_prev = x0;
% u_prev = u0;
% X = [];
% U = [];
% t4 = tic;
% for i = 1:length(v_ref)
%     
%     u = explicit_ctrl_2_2.evaluate(x_prev, 'x.reference', v_ref(i), 'u.previous', u_prev);
%     [temp_t, temp_v] = ode45(@(t,y) dydt_step8(t, y, m, gamma, b, c, g), [0, Ts], [10; x_prev; u]);
%     x = temp_v(end, 2);
%     X = [X x];
%     U = [U u];
%     u_prev = u;
%     x_prev = x;
% end
% T4 = toc(t4);
% 
% x0 = 0.9*alpha;
% u0 = 0; 
% x_prev = x0;
% u_prev = u0;
% X = [];
% U = [];
% t5 = tic;
% for i = 1:length(v_ref)
%     
%     u = explicit_ctrl_3_3.evaluate(x_prev, 'x.reference', v_ref(i), 'u.previous', u_prev);
%     [temp_t, temp_v] = ode45(@(t,y) dydt_step8(t, y, m, gamma, b, c, g), [0, Ts], [10; x_prev; u]);
%     x = temp_v(end, 2);
%     X = [X x];
%     U = [U u];
%     u_prev = u;
%     x_prev = x;
% end
% T5 = toc(t5);
% 
% x0 = 0.9*alpha;
% u0 = 0; 
% x_prev = x0;
% u_prev = u0;
% X = [];
% U = [];
% t6 = tic;
% for i = 1:length(v_ref)
%     
%     u = explicit_ctrl_4_4.evaluate(x_prev, 'x.reference', v_ref(i), 'u.previous', u_prev);
%     [temp_t, temp_v] = ode45(@(t,y) dydt_step8(t, y, m, gamma, b, c, g), [0, Ts], [10; x_prev; u]);
%     x = temp_v(end, 2);
%     X = [X x];
%     U = [U u];
%     u_prev = u;
%     x_prev = x;
% end
% T6 = toc(t6);

t4 = tic;
loop_explicit_2_2 = ClosedLoop(explicit_ctrl_2_2, sys);
data_explicit_2_2 = loop_explicit_2_2.simulate(x0, Nsim, 'x.reference', xref, 'u.previous', u_0);
T4 = toc(t4);
t5 = tic;
loop_explicit_3_3 = ClosedLoop(explicit_ctrl_3_3, sys);
data_explicit_3_3 = loop_explicit_3_3.simulate(x0, Nsim, 'x.reference', xref, 'u.previous', u_0);
T5 = toc(t5);
t6 = tic;
loop_explicit_4_4 = ClosedLoop(explicit_ctrl_4_4, sys);
data_explicit_4_4 = loop_explicit_4_4.simulate(x0, Nsim, 'x.reference', xref, 'u.previous', u_0);
T6 = toc(t6);

clear t1 t2 t3 t4 t5 t6