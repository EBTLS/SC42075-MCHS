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
Np = 7;
Nc = 4;

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
