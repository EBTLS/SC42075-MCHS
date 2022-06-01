clear
clc
close all

%% Define Variables
%  basic variables
syms v u

% basics functions
f1 = 2.4733*u - 0.0108*v + v;
f2 = 1.6880*u - 0.0108*v + v;
f3 = 1.6880*u - 0.0469*v + 1.0409 + v;
f4 = 1.2812*u - 0.0469*v + 1.0409 + v;

% some known parameters
vmax = 57.7150;
vmin = 0;
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
epsilon = 0.001;

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

% f_original =  (3203*u)/2500 - (10409*d2)/10000 + (9531*v)/10000 
%             + (7853*d1*u)/10000 + (1017*d3*u)/2500 + (361*d2*v)/10000 + 10409/10000
% after expand you can find there are only multiply:
% d1*u, d3*u, d2*v

% subs binary variables * real-value variables
% d1*u -> z1
% d2*v -> z2
% d3*u -> z3
f = subs(f_original, [d1*u, d2*v, d3*u], [z1, z2, z3]);
pretty(f);

% 3203 u   10409 d2   9531 v   7853 z1   361 z2   1017 z3   10409
% ------ - -------- + ------ + ------- + ------ + ------- + -----
%  2500      10000     10000    10000     10000     2500    10000

% Convert to standard MLD
MLD.A = 9531/10000;
MLD.B1 = 3203/2500;
MLD.B2 = [0, 10409/10000, 0];
MLD.B3 = [7853/10000, 361/10000, 1017/2500];
MLD.constant = 10409/10000;

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
temp_g = z1 <= M * d1;
g = [g;temp_g];
temp_g = z1 >= m * d1;
g = [g;temp_g];
temp_g = z1 <= u - m * (1 - d1);
g = [g;temp_g];
temp_g = z1 >= u - M * (1 - d1);
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
temp_g = z3 <= M * d3;
g = [g;temp_g];
temp_g = z3 >= m * d3;
g = [g;temp_g];
temp_g = z3 <= u - m * (1 - d3);
g = [g;temp_g];
temp_g = z3 >= u - M * (1 - d3);
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
% E1*x + E2*u + E3*d + E4*z <= g5
MLD.E1 = zeros(n,nu);
MLD.E2 = zeros(n,nv);
MLD.E3 = zeros(n,nd);
MLD.E4 = zeros(n,nz);

% define E1
MLD.E1(1) = 1;
MLD.E1(2) = -1;
MLD.E1(3) = 1;
MLD.E1(4) = -1;
MLD.E1(5) = 1;
MLD.E1(6) = -1;
MLD.E1;

% define E2

% define E3

% define E4

% define g5



% Translate 
% MLD.E1 = 
