function [flag,x,u] = Solution_2_7(Np, Nc, lambda, umax, umin, vmax, vmin, a_comfort, x_0, d_0, z_0, u_0, model, Ts, x_ref)
%Solution_2_7 Solution file for question 2.7
%   Detailed explanation goes here
% Input:
%   Np: prediction horizon
%   Nc: control horizon
%   lambda: relative weight of Jinput in the objective function
%   umax, umin: min and max input
%   vmax, vmin: min and max speed
%   a_comfort: comfortable acceleration limitation
%   x_0: initial x 
%   d_0: initial d
%   z_0: initial z
%   u_0: previous input
%   model: MLD model
%   Ts: sampling time
%   x_ref: reference x
% Output:
%   


%% define parameters
nx = 1; % dimension of x
nu = 1; % dimension of u
nd = 3; % dimension of d
nz = 3; % dimension of z
ng = size(model.g5, 1); % how many inequality constraints

%% prepare target function

c1 = zeros(1, nx*Np); % for x
c2 = zeros(1, nx*Np); % for u
c3 = zeros(1, nd*Np); % for delta
c4 = zeros(1, nz*Np); % for z
c5 = ones(1,Np); % for rho
c6 = 1; % for tau

c = [c1, c2, c3, c4, c5, c6];

%% prepare constraints
% decision variable format:
% [ x_{k+1}, ..., x_{k+Np}, | u_{k}, ..., u_{k+Np-1}, | d_{k+1}, ..., d_{k+Np}, |
%   z_{k+1}, ..., z_{k+Np}, | rho_{1}, ..., rho_{Np}, | tau ]

% state transition
aux_matrix = diag(ones(1,Np-1),-1);
A11 = -kron(aux_matrix, model.A1);
A11 = eye(Np) + A11;

aux_matrix = diag(ones(1, Np), 0);
A12 = -kron(aux_matrix, model.B1);

aux_matrix = diag(ones(1,Np-1),-1);
A13 = -kron(aux_matrix, model.B2);

A14 = -kron(aux_matrix, model.B3);
A15 = zeros(Np,Np);
A16 = zeros(Np,1);

A1 = [A11, A12, A13, A14, A15, A16];
b1 = zeros(Np,1);
b1(1) = model.A1 * x_0 + model.B2 * d_0 + model.B3 * z_0;

% original inequalities
aux_matrix = diag(ones(1, Np), 0);
A21 = kron(aux_matrix, model.E1);

aux_matrix = diag(ones(1, Np-1), 1);
A22 = kron(aux_matrix, model.E2);

aux_matrix = diag(ones(1, Np), 0);
A23 = kron(aux_matrix, model.E3);
A24 = kron(aux_matrix, model.E4);

A25 = zeros(ng*Np,Np);
A26 = zeros(ng*Np,1);

A2 = [A21, A22, A23, A24, A25, A26];
b2 = ones(Np,1);
b2 = kron(b2, model.g5);


% optimization  construction
% for rho
A31 = eye(Np);
A32 = zeros(Np, Np);
A33 = zeros(Np, Np*nd);
A34 = zeros(Np, Np*nz);
A35 = -eye(Np, Np);
A36 = zeros(Np, 1);

A3 = [A31, A32, A33, A34, A35, A36];
b3 = x_ref;

A41 = eye(Np);
A42 = zeros(Np, Np);
A43 = zeros(Np, nd*Np);
A44 = zeros(Np, nz*Np);
A45 = eye(Np, Np);
A46 = zeros(Np, 1);

A4 = [A41, A42, A43, A44, A45, A46];
b4 = x_ref;

% for tau
A51 = zeros(Np, Np);

aux_matrix = - diag(ones(1,Np-1),-1);
A52 = eye(Np) + aux_matrix;

A53 = zeros(Np, nd*Np);
A54 = zeros(Np, nz*Np);
A55 = zeros(Np, Np);
A56 = -ones(Np, 1);

A5 = [A51, A52, A53, A54, A55, A56];
b5 = zeros(Np, 1);
b5(1) = u_0;

A61 = zeros(Np, Np);

aux_matrix = - diag(ones(1,Np-1),-1);
A62 = eye(Np) + aux_matrix;

A63 = zeros(Np, nd*Np);
A64 = zeros(Np, nz*Np);
A65 = zeros(Np, Np);
A66 = ones(Np, 1);

A6 = [A61, A62, A63, A64, A65, A66];
b6 = zeros(Np, 1);
b6(1) = u_0;

% comfortable acceleration
aux_matrix = - diag(ones(1,Np-1),-1);
A71 = eye(Np) + aux_matrix;
A72 = zeros(Np, Np);
A73 = zeros(Np, nd*Np);
A74 = zeros(Np, nz*Np);
A75 = zeros(Np, Np);
A76 = zeros(Np, 1);

A7 = [A71, A72, A73, A74, A75, A76];
b7 = zeros(Np, 1);
b7(1) =x_0;
b7 = b7 + a_comfort * Ts * ones(Np, 1);

aux_matrix = - diag(ones(1,Np-1),-1);
A81 = eye(Np) + aux_matrix;
A82 = zeros(Np, Np);
A83 = zeros(Np, nd*Np);
A84 = zeros(Np, nz*Np);
A85 = zeros(Np, Np);
A86 = zeros(Np, 1);

A8 = [A81, A82, A83, A84, A85, A86];
b8 = zeros(Np, 1);
b8(1) =x_0;
b8 = b8 - a_comfort * Ts * ones(Np, 1);

% prediction horizon vs control horizon

A91 = zeros(Np, Np);
A92 = zeros(Np, Np);
aux_matrix = zeros(Np, Np);
if (Nc < Np)
    aux_matrix([Nc+1: 1: Np], Nc) = 1;
    A92([Nc+1: 1: Np], [Nc+1: 1: Np]) = 1;
    flag = 0;
elseif (Nc > Np)
    fprintf("illegal Nc and Np");
    flag = -1;
else
    flag = 1;
end
A92 = A92 - aux_matrix;
A93 = zeros(Np, nd*Np);
A94 = zeros(Np, nz*Np);
A95 = zeros(Np, Np);
A96 = zeros(Np, 1);

A9 = [A91, A92, A93, A94, A95, A96];
b9 = zeros(Np, 1);

flag = 0 ;

% combined them together
% A = [A1; A2; A3; A4; A5; A6; A7; A8; A9];
% b = [b1; b2; b3; b4; b5; b6; b7; b8; b9];

A = [A1; A2;];
b = [b1; b2;];


%% prepare lb and ub

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


%% prepare solver parameter

% constraints characters
% ctype =repelem(['S','U','U','L','U','L','U','L','S'],...
%                [nx*Np, ng*Np, nx*Np, nx*Np, nu*Np, nu*Np, nx*Np, nx*Np, nu*Np]);

ctype =repelem(['S','U'],...
               [nx*Np, ng*Np]);


% types of the variables
% vartype = repelem(['C','C','B','C','C','C'], ...
%                  [nx*Np, nu*Np, nd*Np, nz*Np, Np, 1]);
vartype = repelem(['C','C','B','C','C','C'], ...
                 [nx*Np, nu*Np, nd*Np, nz*Np, Np, 1]);


% this is a minize question
sense = 1;

% solver options
param.msglev = 3;
param.lpsolver = 2;

%% call solver

if flag == -1
   
    ;
    
else

    [xopt, fopt, status, extra] = glpk (c, A, b, lb, ub, ctype, vartype, sense, param);

    if (status == 2)

        fprintf("feasible solution exists");
        
        fprintf("optimal objective function: d%", fopt);
        
        x = xopt;
        u = xopt([Np+1: 1: Np+Np]);
        flag = 1;
        
    elseif (status == 5)

        fprintf("optimal solution exists");
        
        fprintf("optimal objective function: d%", fopt);
        
        x = xopt;
        u = xopt([Np+1: 1: Np+Np]);
        flag = 1;

    else
        
        fprintf("no feasible solution exists");
        x = Inf;
        u = Inf;
        
        
    end
    
end

end
