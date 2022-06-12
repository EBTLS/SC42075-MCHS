function [explicit_ctrl, sys] = Solution_2_10(Np, Nc, lambda, umax, umin, vmax, vmin, a_comfort, model, Ts, mode)
%Solution_2_10 Solution file for question 2.7
%   Detailed explanation goes here
% Input:
%   Np: prediction horizon
%   Nc: control horizon
%   lambda: relative weight of Jinput in the objective function
%   umax, umin: min and max input
%   vmax, vmin: min and max speed
%   a_comfort: comfortable acceleration limitation
%   model: MLD model
%   Ts: sampling time
%   mode: 1 for implicit, 0 for explicit
% Output:
%   flag: 1 for feasible optimal solution
%

%% define parameters

v1 = 15;
v2 = 28.8575;
v3 = 30;

%% define target function


%% define system model

% % regard [x, d, z] as the whole state variables
sys = Model_generator(vmin, v1, v2, v3, vmax, Ts);

%% define MPC model
% define penalty: the target function

sys.u.min = umin;
sys.u.max = umax;
sys.x.min = vmin;
sys.x.max = vmax;
% sys.y.min = vmin;
% sys.y.max = vmax;

sys.u.with('deltaPenalty');
sys.u.deltaPenalty = InfNormFunction(lambda);

sys.x.with('reference');
sys.x.reference = 'free';
sys.x.penalty = OneNormFunction(1);

sys.x.with('deltaMin');
sys.x.with('deltaMax');
sys.x.deltaMin = -Ts * a_comfort;
sys.x.deltaMax = Ts * a_comfort;

% define Np and Nc horizon
sys.u.with( 'block' );
sys.u.block.from = Nc;
sys.u.block.to = Np;

%% call solver
ctrl = MPCController(sys, Np);

if mode == 'implicit'
    explicit_ctrl = ctrl;    
elseif mode == 'block'
    explicit_ctrl = ctrl.toExplicit();
    figure
    explicit_ctrl.partition.plot();    
end

end

