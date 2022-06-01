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
syms d1 d2 d3 d4
syms d5 d6 d7

% auxiliary real-value variables
syms z1 z2 z3 z4 z5 z6 z7 z8 z9 z10 z11 z12 z13 z14

% machine precision value
epsilon = 0.001;


%% Construct Target Function
% target function
f_original = d1*(1-d2)*f1 + d2*(1-d3)*f2 + d3*(1-d4)*f3 + d4*f4;
f = expand(f_original);

% subs binary variables products
% d1d2 -> d5
% d2d3 -> d6
% d3d4 -> d7
f = subs(f,d1*d2, d5);
f = subs(f,d2*d3, d6);
f = subs(f,d3*d4, d7);


% subs binary variables * real-value variables
% d1*u -> z1
% d1*v -> z2
% d2*u -> z3
% d2*v -> z4
% d3*u -> z5
% d3*v -> z6
% d4*u -> z7
% d4*v -> z8
% d5*u -> z9
% d5*v -> z10
% d6*u -> z11
% d6*v -> z12
% d7*u -> z13
% d7*v -> z14
f = subs(f, d1*u, z1);
f = subs(f, d1*v, z2);
f = subs(f, d2*u, z3);
f = subs(f, d2*v, z4);
f = subs(f, d3*u, z5);
f = subs(f, d3*v, z6);
f = subs(f, d4*u, z7);
f = subs(f, d4*v, z8);
f = subs(f, d5*u, z9);
f = subs(f, d5*v, z10);
f = subs(f, d6*u, z11);
f = subs(f, d6*v, z12);
f = subs(f, d7*u, z13);
f = subs(f, d7*v, z14);
pretty(f)

%% construct constraints

constraints = [];

% first-order delta variables

g = [];
temp_g = (v - m) <= M * (1 - d1);
g = [g;temp_g];
temp_g = (v - m) >= epsilon + (m - epsilon) * d1;
g = [g;temp_g];
constraints = [constraints ; g];

g = [];
temp_g = (v - v1) <= M * (1 - d2);
g = [g;temp_g];
temp_g = (v - v1) >= epsilon + (m - epsilon) * d2;
g=[g;temp_g];
constraints = [constraints ; g];

g = [];
temp_g = (v - v2) <= M * (1 - d3);
g = [g;temp_g];
temp_g = (v - v2) >= epsilon + (m - epsilon) * d3;
g=[g;temp_g];
constraints = [constraints ; g];

g = [];
temp_g = (v - v3) <= M * (1 - d4);
g = [g;temp_g];
temp_g = (v - v3) >= epsilon + (m - epsilon) * d4;
g=[g;temp_g];
constraints = [constraints ; g];

% second-order delta variables
% d1d2 -> d5
% d2d3 -> d6
% d3d4 -> d7
g = []; 
temp_g = (-d1 + d5) <= 0;
g = [g; temp_g];
temp_g = (-d2 + d5) <= 0;
g = [g; temp_g];
temp_g = (d1 + d2 - d5) <= 0;
g = [g; temp_g];
constraints = [constraints ; g];

g = []; 
temp_g = (-d2 + d6) <= 0;
g = [g; temp_g];
temp_g = (-d3 + d6) <= 0;
g = [g; temp_g];
temp_g = (d2 + d3 - d6) <= 0;
g = [g; temp_g];
constraints = [constraints ; g];

g = []; 
temp_g = (-d3 + d7) <= 0;
g = [g; temp_g];
temp_g = (-d4 + d7) <= 0;
g = [g; temp_g];
temp_g = (d3 + d4 - d7) <= 0;
g = [g; temp_g];
constraints = [constraints ; g];

% delta * real-value variables: z
% d1*u -> z1
% d1*v -> z2
% d2*u -> z3
% d2*v -> z4
% d3*u -> z5
% d3*v -> z6
% d4*u -> z7
% d4*v -> z8
% d5*u -> z9
% d5*v -> z10
% d6*u -> z11
% d6*v -> z12
% d7*u -> z13
% d7*v -> z14
g = [];
temp_g = z1 <= M * d1;
g = [g; temp_g];
temp_g = z1 >= m * d1;
g = [g; temp_g];
temp_g = z1 <= u - m * (1 - d1);
g = [g; temp_g];
temp_g = z1 >= u - M * (1 - d1);
g = [g; temp_g];
constraints = [constraints ; g];

g = [];
temp_g = z2 <= M * d1;
g = [g; temp_g];
temp_g = z2 >= m * d1;
g = [g; temp_g];
temp_g = z2 <= v - m * (1 - d1);
g = [g; temp_g];
temp_g = z2 >= v - M * (1 - d1);
g = [g; temp_g];
constraints = [constraints ; g];

g = [];
temp_g = z3 <= M * d1;
g = [g; temp_g];
temp_g = z3 >= m * d1;
g = [g; temp_g];
temp_g = z3 <= u - m * (1 - d1);
g = [g; temp_g];
temp_g = z3 >= u - M * (1 - d1);
g = [g; temp_g];
constraints = [constraints ; g];

g = [];
temp_g = z1 <= M * d1;
g = [g; temp_g];
temp_g = z1 >= m * d1;
g = [g; temp_g];
temp_g = z1 <= u - m * (1 - d1);
g = [g; temp_g];
temp_g = z1 >= u - M * (1 - d1);
g = [g; temp_g];
constraints = [constraints ; g];

g = [];
temp_g = z1 <= M * d1;
g = [g; temp_g];
temp_g = z1 >= m * d1;
g = [g; temp_g];
temp_g = z1 <= u - m * (1 - d1);
g = [g; temp_g];
temp_g = z1 >= u - M * (1 - d1);
g = [g; temp_g];
constraints = [constraints ; g];

g = [];
temp_g = z1 <= M * d1;
g = [g; temp_g];
temp_g = z1 >= m * d1;
g = [g; temp_g];
temp_g = z1 <= u - m * (1 - d1);
g = [g; temp_g];
temp_g = z1 >= u - M * (1 - d1);
g = [g; temp_g];
constraints = [constraints ; g];

g = [];
temp_g = z1 <= M * d1;
g = [g; temp_g];
temp_g = z1 >= m * d1;
g = [g; temp_g];
temp_g = z1 <= u - m * (1 - d1);
g = [g; temp_g];
temp_g = z1 >= u - M * (1 - d1);
g = [g; temp_g];
constraints = [constraints ; g];

g = [];
temp_g = z1 <= M * d1;
g = [g; temp_g];
temp_g = z1 >= m * d1;
g = [g; temp_g];
temp_g = z1 <= u - m * (1 - d1);
g = [g; temp_g];
temp_g = z1 >= u - M * (1 - d1);
g = [g; temp_g];
constraints = [constraints ; g];



