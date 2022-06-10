function [flag, explicit_ctrl] = Solution_2_10(Np, Nc, lambda, umax, umin, vmax, vmin, a_comfort, v_0, u_0, model, Ts, v_ref)
%Solution_2_10 Solution file for question 2.7
%   Detailed explanation goes here
% Input:
%   Np: prediction horizon
%   Nc: control horizon
%   lambda: relative weight of Jinput in the objective function
%   umax, umin: min and max input
%   vmax, vmin: min and max speed
%   a_comfort: comfortable acceleration limitation
%   v_0: initial v 
%   u_0: previous input
%   model: MLD model
%   Ts: sampling time
%   v_ref: reference v
% Output:
%   flag: 1 for feasible optimal solution
%



%% define parameters
nx = 1; % dimension of x
nu = 1; % dimension of u
nd = 3; % dimension of d
nz = 3; % dimension of z
ng = size(model.g5, 1); % how many inequality constraints

v1 = 15;
v2 = 28.8575;
v3 = 30;


%% variables classification


%% define target function
% output variable (u) format:
% [ x_{k+1}, ..., x_{k+Np}, | u_{k}, ..., u_{k+Np-1}, | d_{k+1}, ..., d_{k+Np}, |
%   z_{k+1}, ..., z_{k+Np}, | rho_{1}, ..., rho_{Np}, | tau ]

f1 = zeros(1, nx*Np); % for x
f2 = zeros(1, nx*Np); % for u
f3 = zeros(1, nd*Np); % for delta
f4 = zeros(1, nz*Np); % for z
f5 = ones(1, Np); % for rho
f6 = lambda; % for tau

f = [f1, f2, f3, f4, f5, f6];


%% define equality constraints
% input variable (th) format:
% [u_{k-1}; x_{k}; d_{k}; z_{k}]

% state transition
aux_matrix = diag(ones(1,Np-1),-1);
Ae11 = -kron(aux_matrix, model.A1);
Ae11 = eye(Np) + Ae11;

aux_matrix = diag(ones(1, Np), 0);
Ae12 = -kron(aux_matrix, model.B1);

aux_matrix = diag(ones(1,Np-1),-1);
Ae13 = -kron(aux_matrix, model.B2);

Ae14 = -kron(aux_matrix, model.B3);
Ae15 = zeros(Np,Np);
Ae16 = zeros(Np,1);

Ae1 = [Ae11, Ae12, Ae13, Ae14, Ae15, Ae16];
be1 = ones(Np,1) * model.constant;
pE1 = ones(Np, nu + nx + nd + nz) * 0;
pE1(1,[nu+1: 1: nu+nx+nd+nz]) = [model.A1, model.B2, model.B3];

% prediction horizon vs control horizon
Ae21 = zeros(Np, Np);
Ae22 = zeros(Np, Np);
aux_matrix = zeros(Np, Np);
if (Nc < Np)
    aux_matrix([Nc+1: 1: Np], Nc) = 1;
    Ae22([Nc+1: 1: Np], [Nc+1: 1: Np]) = 1;
    flag = 0;
elseif (Nc > Np)
    fprintf("illegal Nc and Np");
    flag = -1;
else
    flag = 1;
end
Ae22 = Ae22 - aux_matrix;
Ae23 = zeros(Np, nd*Np);
Ae24 = zeros(Np, nz*Np);
Ae25 = zeros(Np, Np);
Ae26 = zeros(Np, 1);

Ae2 = [Ae21, Ae22, Ae23, Ae24, Ae25, Ae26];
be2 = zeros(Np, 1);
pE2 = ones(Np, nu + nx + nd + nz) * 0;

Ae = [Ae1; Ae2];
be = [be1; be2];
pE = [pE1; pE2];


%% define inequality constraints
% input variable (th) format:
% [ u_{k-1}; x_{k}; d_{k}; z_{k}]

% original inequalities
aux_matrix = diag(ones(1, Np), 0);
A11 = kron(aux_matrix, model.E1);

aux_matrix = diag(ones(1, Np-1), 1);
aux_matrix(end,end) = 1;
A12 = kron(aux_matrix, model.E2);

aux_matrix = diag(ones(1, Np), 0);
A13 = kron(aux_matrix, model.E3);
A14 = kron(aux_matrix, model.E4);

A15 = zeros(ng*Np,Np);
A16 = zeros(ng*Np,1);

A1 = [A11, A12, A13, A14, A15, A16];
b1 = ones(Np,1);
b1 = kron(b1, model.g5);
pB1 = ones(Np*ng, nu + nx + nd + nz) * 0;

% optimization  construction
% for rho
A21 = eye(Np);
A22 = zeros(Np, Np);
A23 = zeros(Np, Np*nd);
A24 = zeros(Np, Np*nz);
A25 = -eye(Np, Np);
A26 = zeros(Np, 1);

A2 = [A21, A22, A23, A24, A25, A26];
b2 = v_ref;
pB2 = ones(Np, nu + nx + nd + nz) * 0;

A31 = eye(Np);
A32 = zeros(Np, Np);
A33 = zeros(Np, nd*Np);
A34 = zeros(Np, nz*Np);
A35 = eye(Np, Np);
A36 = zeros(Np, 1);

A3 = [A31, A32, A33, A34, A35, A36];
b3 = v_ref;
pB3 = ones(Np, nu + nx + nd + nz) * 0;

% for tau
A41 = zeros(Np, Np);

aux_matrix = - diag(ones(1,Np-1),-1);
A42 = eye(Np) + aux_matrix;

A43 = zeros(Np, nd*Np);
A44 = zeros(Np, nz*Np);
A45 = zeros(Np, Np);
A46 = -ones(Np, 1);

A4 = [A41, A42, A43, A44, A45, A46];
b4 = zeros(Np, 1);
pB4 = ones(Np, nu + nx + nd + nz) * 0;
pB4(1,[1: 1: nu]) = 1;

A51 = zeros(Np, Np);

aux_matrix = - diag(ones(1,Np-1),-1);
A52 = eye(Np) + aux_matrix;

A53 = zeros(Np, nd*Np);
A54 = zeros(Np, nz*Np);
A55 = zeros(Np, Np);
A56 = ones(Np, 1);

A5 = [A51, A52, A53, A54, A55, A56];
b5 = zeros(Np, 1);
pB5 = ones(Np, nu + nx + nd + nz) * 0;
pB5(1,[1: 1: nu]) = 1;

% comfortable acceleration
aux_matrix = - diag(ones(1,Np-1),-1);
A61 = eye(Np) + aux_matrix;
A62 = zeros(Np, Np);
A63 = zeros(Np, nd*Np);
A64 = zeros(Np, nz*Np);
A65 = zeros(Np, Np);
A66 = zeros(Np, 1);

A6 = [A61, A62, A63, A64, A65, A66];
b6 = zeros(Np, 1);
pB6 = ones(Np, nu + nx + nd + nz) * 0;
pB6(1, [nu+1: 1: nu+nx]) = 1;

aux_matrix = - diag(ones(1,Np-1),-1);
A71 = eye(Np) + aux_matrix;
A72 = zeros(Np, Np);
A73 = zeros(Np, nd*Np);
A74 = zeros(Np, nz*Np);
A75 = zeros(Np, Np);
A76 = zeros(Np, 1);

A7 = [A71, A72, A73, A74, A75, A76];
b7 = zeros(Np, 1);
pB7 = ones(Np, nu + nx + nd + nz) * 0;
pB7(1, [nu+1: 1: nu+nx]) = 1;

A = [A1; A2; -A3; A4; -A5; A6; -A7];
b = [b1; b2; -b3; b4; -b5; b6; -b7];
pB = [pB1 ;pB2; -pB3; pB4; -pB5; pB6; -pB7];

%% prepare lb and ub
% output variable (u) format:
% [ x_{k+1}, ..., x_{k+Np}, | u_{k}, ..., u_{k+Np-1}, | d_{k+1}, ..., d_{k+Np}, |
%   z_{k+1}, ..., z_{k+Np}, | rho_{1}, ..., rho_{Np}, | tau ]

% state lb
lb1 = ones(Np, 1) * vmin;
% input lb
lb2 = ones(Np, 1) * umin;
% d lb
lb3 = 0 * ones(Np*nd, 1);
% z lb
aux_matrix = ones(Np,1);
lb4 = kron(aux_matrix, [0; 0; 0]);
% rho lb
lb5 = 0 * ones(Np ,1);
% tau lb
lb6 = 0 * ones(1);

% state ub
ub1 = ones(Np, 1) * vmax;
% input ub
ub2 = ones(Np, 1) * umax;
% d ub
ub3 = 1 * ones(Np*nd, 1);
% z ub
aux_matrix = ones(Np,1);
ub4 = kron(aux_matrix, [umax; vmax; umax]);
% rho lb
ub5 = +Inf * ones(Np, 1);
% tau lb
ub6 = +Inf * ones(1);

% combine the ub and lb
lb = [lb1; lb2; lb3; lb4; lb5; lb6];
ub = [ub1; ub2; ub3; ub4; ub5; ub6];


%% variable type definition
vartype = repelem(['C','C','B','C','C','C'], ...
                 [nx*Np, nu*Np, nd*Np, nz*Np, Np, 1]);

%% call solver

opt = Opt('f', f, 'A', A, 'b', b, 'pB', pB, 'Ae', Ae, 'be', be, 'pE', pE, ...
        'lb', lb, 'ub', ub, 'vartype', vartype);
opt.qp2lcp();
explicit_ctrl = opt.solve();


end

