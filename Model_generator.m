function model = Model_generator(MLD_model, vmin, vmax, umin, umax)
% Model_generator Output an MLD model with 3 delta variables
    S.A = MLD_model.A1;
    S.nx = 1;
    S.nu = 1;
    S.ny = 1;
    S.nz = 3;
    S.nd = 3;

    S.xl = vmin;
    S.xu = vmax;
    S.nxb = 0;

    S.ul = umin;
    S.uu = umax;
    S.nub = 0;

    S.yl = 0;
    S.yu = 0;
    S.nyb = 0;

    S.dl = [0 0 0];
    S.du = [1 1 1];

    S.zl = [umin vmin umin];
    S.zu = [umax vmax umax];

    S.B1 = MLD_model.B1;
    S.B2 = MLD_model.B2;
    S.B3 = MLD_model.B3;
    S.B5 = MLD_model.constant;
    S.C = 0;

    S.D1 = 0;
    S.D2 = 0;
    S.D3 = 0;

    % S:
    % E2*d + E3*z <= E1*u + E4*v + E5
    % MLD:
    % E1*v + E2*u + E3*d + E4*z <= g5

    S.E1 = -MLD_model.E2;
    S.E2 = MLD_model.E3;
    S.E3 = MLD_model.E4;
    S.E4 = -MLD_model.E1;
    S.E5 = MLD_model.g5;

    flag = 1;
    if flag
        model = MLDSystem(S)
    else
        model = MLDSystem('pwa_mld.hys')
    end
end

