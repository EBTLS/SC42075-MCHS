%%

clear 
clc
close all

%%2.6
syms u v

F.f1 = 2.473262032*u - 0.01115077450*v;
F.f2 = 1.687956204*u - 0.01115077450*v;
F.f3 = 1.687956204*u - 0.04831986298*v + 1.105233996;
F.f4 = 1.281163435*u - 0.04831986298*v + 1.105233996;


syms d1 d2 d3 

F.f = (1 - d1)*F.f1 + d1*(1 - d2)*F.f2 + d2*(1 - d3)*F.f3 + d3*F.f4;
% subs(F.f, [d1, d2, d3], [1, 0, 0]) - F.f2

syms d4 d5

% F.f = (1 - d1)*F.f1 + d1*F.f2 + d2*F.f3 + d3*F.f4 - d4*F.f2 - d5*F.f3;

temp = subs(expand(F.f), [d1*d2, d2*d3, d1*d2*v, d2*d3*v, d1*d2*u, d2*d3*u], [d4, d5, d4*v, d5*v, d4*u, d5*u]);

syms z1 z2 z3 z4 z5 z6 z7 z8 z9 z10

temp = subs(temp, [d1*u, d2*u, d3*u, d4*u, d5*u], [z6, z7, z8, z9, z10]);
F.f = subs(temp, [d1*v, d2*v, d3*v, d4*v, d5*v], [z1, z2, z3, z4, z5]);
F.f 
clear temp

