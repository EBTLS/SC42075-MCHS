function model = Model_generator(vmin, v1, v2, v3, vmax, Ts)
% Model_generator Output an MLD model with 3 delta variables


A_1 = 1 - 0.0108 * Ts;
A_2 = A_1;
A_3 = 1 - 0.0469 * Ts;
A_4 = A_3;
B_1 = 2.4733 * Ts;
B_2 = 1.6880 * Ts;
B_3 = 1.6880 * Ts;
B_4 = 1.2812 * Ts;
C = 0;
f = 1.0409 * Ts;

sys1 = LTISystem('A', A_1, 'B', B_1, 'f', 0);
R_1 = Polyhedron('lb', vmin, 'ub', v1);
sys1.setDomain('x', R_1);

sys2 = LTISystem('A', A_2, 'B', B_2, 'f', 0);
R_2 = Polyhedron('lb', v1, 'ub', v2);
sys2.setDomain('x', R_2);

sys3 = LTISystem('A', A_3, 'B', B_3, 'f', f);
R_3 = Polyhedron('lb', v2, 'ub', v3);
sys3.setDomain('x', R_3);

sys4 = LTISystem('A', A_4, 'B', B_4, 'f', f);
R_4 = Polyhedron('lb', v3, 'ub', vmax);
sys4.setDomain('x', R_4);

model = PWASystem([sys1, sys2, sys3, sys4]);

%     S.A = MLD_model.A1;
%     S.nx = 1;
%     S.nu = 1;
%     S.ny = 1;
%     S.nz = 3;
%     S.nd = 3;
% 
%     S.xl = vmin;
%     S.xu = vmax;
%     S.nxb = 0;
% 
%     S.ul = umin;
%     S.uu = umax;
%     S.nub = 0;
% 
%     S.yl = 0;
%     S.yu = 0;
%     S.nyb = 0;
% 
%     S.dl = [0 0 0];
%     S.du = [1 1 1];
% 
%     S.zl = [umin vmin umin];
%     S.zu = [umax vmax umax];
% 
%     S.B1 = MLD_model.B1;
%     S.B2 = MLD_model.B2;
%     S.B3 = MLD_model.B3;
%     S.B5 = MLD_model.constant;
%     S.C = 0;
% 
%     S.D1 = 0;
%     S.D2 = 0;
%     S.D3 = 0;
% 
%     % S:
%     % E2*d + E3*z <= E1*u + E4*v + E5
%     % MLD:
%     % E1*v + E2*u + E3*d + E4*z <= g5
% 
%     S.E1 = -MLD_model.E2;
%     S.E2 = MLD_model.E3;
%     S.E3 = MLD_model.E4;
%     S.E4 = -MLD_model.E1;
%     S.E5 = MLD_model.g5;
% 
%     flag = 0;
%     if flag
%         model = MLDSystem(S)
%     else
%         model = MLDSystem('pwa_mld.hys')
%         model = mpt_sys('pwa_mld.hys')
%     end


end

