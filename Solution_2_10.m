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

%% define target function


%% define system model

% % regard [x, d, z] as the whole state variables
sys = LTISystem('A', [model.A1, model.B3, model.B4], 'B', model.B2);


%% define MPC model

% define penalty: the target function

Q = zeros(1, nx + nd + nz);
Q([1: 1: nx]) = 1;
sys.x.penalty = OneNormFunction(Q);

R = lambda;
sys.u.Penalty = 0;
sys.u.deltaPenalty = InfNormFunction(lambda);


% define Np and Nc horizon
sys.u.with( 'block' );
sys.u.block.from = Nc;
sys.u.block.to = Np;

% define constraints

% relations between v u z and d


% comfortable acceleration constraints
sys.x.with('deltaMin');
sys.x.with('deltaMax');
x_delta_min = -Inf * ones(nx + nd + nz, 1);
x_delta_min([1: 1: nx]) = -Ts * a_comfort;
x_delta_max = Inf * ones(nx + nd + nz, 1);
x_delta_max([1: 1: nx]) = Ts * a_comfort;
sys.x.deltaMin = x_delta_min;
sys.x.deltaMax = x_delta_max;

% define binary variables
% sys.x.with('binary')
% sys.x.binary = [nx+1: 1: nx+nd];


% define lower bound and upper bound
% xmin = [vmin * ones(1 , nx); 0 * ones(1, nd); [umin; vmin; umin]];
% sys.x.min = xmin;
% xmax = [vmax * ones(1 , nx); 1.1 * ones(1, nd); [umax; vmax; umax]];
% sys.x.max = xmax;
% 
% sys.u.min = umin;
% sys.u.max = umax;


xmin = [vmin * ones(nx, 1); 0 * ones(nd, 1)];
sys.x.min = xmin;
xmax = [vmax * ones(nx, 1); 1 * ones(nd, 1)];
sys.x.max = xmax;

sys.u.min = [umin; [umin; vmin; umin]];
sys.u.max = [umax; [umax; vmax; umax]];


%% call solver
ctrl = MPCController(sys, Np);

explicit_ctrl = ctrl.toExplicit();
explicit_ctrl.partition.plot();



end

