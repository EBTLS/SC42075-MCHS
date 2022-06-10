function dydt = dydt_step8(t, y, m, gamma, b, c, g)
%DYDT_STEP8 the function is used to continuous-time simulation of step 8
%  
% input:
%   t,y: parameter for ode function
%   m,gamma,b,c: system parameters
%   g: possible gear settings


%% some parameters 

v1 = 15;
v2 = 28.8575;
v3 = 30;


%% dydt generate
    
if y < v1
    
    gear = g(1);
    
elseif y<v3
    
    gear = g(2);
    
else 
    
    gear = g(3);

end


dydt = zeros(3,1);

dydt(1) = y(2);
dydt(2) = 1/m * b/(1 + gamma * gear) * y(3) - 1/m * c * y(2)^2; 
dydt(3) = 0;  % represent u



end

