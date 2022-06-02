function model = MLD_Model_3delta()
%MLD_Model_3delta Output an MLD model with 3 delta variables

%% Define Variables
%  basic variables
syms v u

% basics functions
f1 = 0.15* (2.4733*u - 0.0108*v) + v;
f2 = 0.15* (1.6880*u - 0.0108*v) + v;
f3 = 0.15* (1.6880*u - 0.0469*v + 1.0409) + v;
f4 = 0.15* (1.2812*u - 0.0469*v + 1.0409) + v;

% some known parameters
vmax = 57.7150;
vmin = 0;
umax = 1.3;
umin = -1.3;
v1 = 15;
v2 = 28.8575;
v3 = 30;

m = vmin;
M = vmax;

% binary auxiliary variables
% d1 -> v <= v1
% d2 -> v <= v2
% d3 -> v <= v3
syms d1 d2 d3

% auxiliary real-value variables
syms z1 z2 z3

% machine precision value
epsilon = 0.0000;

% Discrete Time Step
T = 0.15;

% dimensions
nv = 1;
nu = 1;
nz = 3;
nd = 3;

n = nv + nu + nz * 4 + nd * 2;

%% Construct Target Function
% target function
f_original = d1 * (f1 - f2) + d2 * (f2 - f3) + d3 * (f3 - f4) + f4;
f_original = expand(f_original);

% f_original =  (9609*u)/50000 - (31227*d2)/200000 + (198593*v)/200000 
%               + (23559*d1*u)/200000 + (3051*d3*u)/50000 + (1083*d2*v)/200000 + 31227/200000

% after expand you can find there are only multiply:
% d1*u, d3*u, d2*v

% subs binary variables * real-value variables
% d1*u -> z1
% d2*v -> z2
% d3*u -> z3
f = subs(f_original, [d1*u, d2*v, d3*u], [z1, z2, z3]);
pretty(f);

% 9609 u   31227 d2   198593 v   23559 z1   1083 z2   3051 z3    31227
% ------ - -------- + -------- + -------- + ------- + ------- + ------
%  50000    200000     200000     200000     200000    50000    200000

% Convert to standard MLD
MLD.A1 = 198593/200000;
MLD.B1 = 9609/50000;
MLD.B2 = [0, 31227/200000, 0];
MLD.B3 = [23559/200000, 1083/200000, 3051/50000];
MLD.constant = 31227/200000;

%% construct constraints

constraints = [];

% delta variables constraints
% d1 -> v - v1 <= 0 
g = [];
temp_g = (v - v1) <= M * (1 - d1);
g = [g;temp_g];
temp_g = (v - v1) >= epsilon + (m - epsilon) * d1;
g = [g;temp_g];
constraints = [constraints ; g];
fprintf("d1 -> v - v1 <= 0\n");
pretty(g);

% d2 -> v - v2 <= 0 
g = [];
temp_g = (v - v2) <= M * (1 - d2);
g = [g;temp_g];
temp_g = (v - v2) >= epsilon + (m - epsilon) * d2;
g = [g;temp_g];
constraints = [constraints ; g];
fprintf("d2 -> v - v2 <= 0\n");
pretty(g);

% d3 -> v - v3 <= 0 
g = [];
temp_g = (v - v3) <= M * (1 - d3);
g = [g;temp_g];
temp_g = (v - v3) >= epsilon + (m - epsilon) * d3;
g = [g;temp_g];
constraints = [constraints ; g];
fprintf("d3 -> v - v3 <= 0\n");
pretty(g);

% z variables constraints
% d1*u -> z1
g = [];
temp_g = z1 <= umax * d1;
g = [g;temp_g];
temp_g = z1 >= umin * d1;
g = [g;temp_g];
temp_g = z1 <= u - umin * (1 - d1);
g = [g;temp_g];
temp_g = z1 >= u - umax * (1 - d1);
g = [g;temp_g];
constraints = [constraints ; g];
fprintf("d1*u -> z1\n");
pretty(g);

% d2*v -> z2
g = [];
temp_g = z2 <= M * d2;
g = [g;temp_g];
temp_g = z2 >= m * d2;
g = [g;temp_g];
temp_g = z2 <= v - m * (1 - d2);
g = [g;temp_g];
temp_g = z2 >= v - M * (1 - d2);
g = [g;temp_g];
constraints = [constraints ; g];
fprintf("d2*v -> z2\n");
pretty(g);

% d3*u -> z3
g = [];
temp_g = z3 <= umax * d3;
g = [g;temp_g];
temp_g = z3 >= umin * d3;
g = [g;temp_g];
temp_g = z3 <= u - umin * (1 - d3);
g = [g;temp_g];
temp_g = z3 >= u - umax * (1 - d3);
g = [g;temp_g];
constraints = [constraints ; g];
fprintf("d3*u -> z3\n");
pretty(g);

% Other Constraints
% speed constraint
g = [];
temp_g = vmin <= v;
g = [g; temp_g];
temp_g = v <= vmax;
g = [g; temp_g];
constraints = [constraints; g];
fprintf("Other Constraints\n");
pretty(g);

clear g temp_g

% comfort constraint
% cannot be modified in a single step

% There are total n = 20 constraint

%% change to standard MLD constraints

ng = nd * 2 + nz * 4;

% E1*v + E2*u + E3*d + E4*z <= g5
MLD.E1 = zeros(ng, nv);
MLD.E2 = zeros(ng, nu);
MLD.E3 = zeros(ng, nd);
MLD.E4 = zeros(ng, nz);
MLD.g5 = zeros(ng, 1);

% define E1, v
MLD.E1(1) = 1;
MLD.E1(2) = -1;
MLD.E1(3) = 1;
MLD.E1(4) = -1;
MLD.E1(5) = 1;
MLD.E1(6) = -1;
MLD.E1(13) = -1;
MLD.E1(14) = 1;

% define E2, u
MLD.E2(9) = -1;
MLD.E2(10) = 1;
MLD.E1(17) = -1;
MLD.E1(18) = 1;

% % define E3, [d1 d2 d3]
% % d1
% MLD.E3(1, 1) = 11543/200;
% MLD.E3(2, 1) = -1/10000;
% % d2
% MLD.E3(3, 2) = 11543/200;
% MLD.E3(4, 2) = -1/10000;
% % d3
% MLD.E3(5, 3) = 11543/200;
% MLD.E3(6, 3) = -1/10000;
% % d1*u -> z1
% MLD.E3(7, 1) = -13/10;
% MLD.E3(8, 1) = -13/10;
% MLD.E3(9, 1) = 13/10;
% MLD.E3(10, 1) = 13/10;
% % d2*v -> z2
% MLD.E3(11, 2) = -11543/200;
% MLD.E3(14, 2) = 11543/200;
% % d3*u -> z3
% MLD.E3(15, 3) = -13/10;
% MLD.E3(16, 3) = -13/10;
% MLD.E3(17, 3) = 13/10;
% MLD.E3(18, 3) = 13/10;

% define E3, [d1 d2 d3]: epsilon = 0
% d1
MLD.E3(1, 1) = 11543/200;
MLD.E3(2, 1) = 0;
% d2
MLD.E3(3, 2) = 11543/200;
MLD.E3(4, 2) = 0;
% d3
MLD.E3(5, 3) = 11543/200;
MLD.E3(6, 3) = 0;
% d1*u -> z1
MLD.E3(7, 1) = -13/10;
MLD.E3(8, 1) = -13/10;
MLD.E3(9, 1) = 13/10;
MLD.E3(10, 1) = 13/10;
% d2*v -> z2
MLD.E3(11, 2) = -11543/200;
MLD.E3(14, 2) = 11543/200;
% d3*u -> z3
MLD.E3(15, 3) = -13/10;
MLD.E3(16, 3) = -13/10;
MLD.E3(17, 3) = 13/10;
MLD.E3(18, 3) = 13/10;

% define E4, [z1 z2 z3]
% d1*u -> z1
MLD.E4(7, 1) = 1;
MLD.E4(8, 1) = -1;
MLD.E4(9, 1) = 1;
MLD.E4(10, 1) = -1;
% d2*v -> z2
MLD.E4(11, 2) = 1;
MLD.E4(12, 2) = -1;
MLD.E4(13, 2) = 1;
MLD.E4(14, 2) = -1;
% d3*u -> z3
MLD.E4(15, 3) = 1;
MLD.E4(16, 3) = -1;
MLD.E4(17, 3) = 1;
MLD.E4(18, 3) = -1;

% define g5, constant: epsilon = 0
MLD.g5(1) = 15 + 11543/200;
MLD.g5(2) = - 15;
% d2
MLD.g5(3) = 11543/400 + 11543/200;
MLD.g5(4) = - 11543/400;
% d3
MLD.g5(5) = 30 + 11543/200;
MLD.g5(6) = - 30;
% d1*u -> z1
MLD.g5(9) = 13/10;
MLD.g5(10) = 13/10;
% d2*v -> z2
MLD.g5(14) = 11543/200;
% d3*u -> z3
MLD.g5(17) = 13/10;
MLD.g5(18) = 13/10;


model = MLD;

end