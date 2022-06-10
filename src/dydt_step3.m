function dydt = dydt_step3(t,y,model,alpha,beta,m,gamma,b,c,vmax)
%DYDT_STEP3 the function is used to continuous-time simulation of step 3
%  
% input:
%   t,y: parameter for ode function
%   model: whether use PWA (1) or Original Model (0)
%   alpha, beta: parameter in the PWA
%   m,gamma,b,c,vmax: system parameters
    
    gear = 1;
    dydt = zeros(2,1);
    
    % generate a sinusoidal throttle input
    u = sin(pi*t);
    
    % calculate dydt
    if model == 0
    % if we want to use the Original Model
    
        dydt(1) = y(2);
        dydt(2) = 1/m * b/(1 + gamma * gear) * u - 1/m * c * y(2)^2; 
        
    elseif model ==1
    % if we want to use the PWA Model
        
        if y(2) <= alpha
        % if lies in the first part
        
            dydt(1) = y(2);
            dydt(2) = 1/m * b/(1 + gamma * gear) * u - 1/m * beta/alpha * y(2);
            
        else
        % if lies in the second part
        
            dydt(1) = y(2);
            dydt(2) = 1/m * b/(1 + gamma * gear) * u -...
                1/m * ((c * vmax^2 - beta)/(vmax - alpha) * (y(2) - vmax) + c * vmax^2);   
            
        end    
    end
end

