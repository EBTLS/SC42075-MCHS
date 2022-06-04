function [x, u] = Simulator_2_8(Np, Nc, lambdam, umax, umin, vmax, vmin, a_comfort, x_0, x_ref, Ts, T_0, T_end, modelc)
%Simulator_2_8:  Simulator to simulate the closed-loop behavior of the system
% Input:
%   Np: prediction horizon
%   Nc: control horizon
%   lambda: relative weight of Jinput in the objective function
%   umax, umin: min and max input
%   vmax, vmin: min and max speed
%   a_comfort: comfortable acceleration limitation
%   x_0: initial x 
%   model: MLD model
%   Ts: sampling time
%   x_ref: reference x
%   T_0: starting time
%   T_end: stop time
%   modelc: continuous time model
% Output:

%% some parameters 

v1 = 15;
v2 = 28.8575;
v3 = 30;

%% judge d_0 and z_0

if x_0 <= v1
    
    d_0 = [1; 1; 1];
    
elseif x_0 <= v2
    
    d_0 = [0; 1; 1];
    
elseif x_0 <=v3
    
    d_0 = [0; 0; 1];
    
else
    
    d_0 = [0; 0; 0];
    
end

z_0 = d_0 .* [u_0; x_0; u_0];

%% start simulation

x_history = zeros(1, length(T_0: Ts: T_end) + 1);
u_history = zeros(1, length(T_0: Ts: T_end));

i = 1;
x_history(i) = x_0;
u_history(i) = u_0;

for t = T_0: Ts: T_end
    
    if (t + (Np - 1) * Ts > T_end)
    % if Np future > T_end
    
        temp_x_ref = x_ref([i: 1: end]);
        temp_x_ref = x_ref(end) * ones(); %% TO DO: to be finished
        
    else
        
        temp_x_ref = x_ref([i: 1: i + Np - 1])
        
        
    end
    
    
    [flag, x, u] = Solution_2_7(Np, Nc, lambda, umax, umin, vmax, vmin, a_comf_max,... 
                x_0, u_0, model, Ts, x_ref(i))
            
    
            
    if flag == 1
        
        % update state with continuous model
        [temp_t, temp_x] = ode45(model, [0,Ts],x_0);
        
    else
    
        fprintf("no feasible optimal solution at time d% \n", t);
        break;
       
    end
    
    % update state and input 
    x_0 = temp_x;
    u_0 = u(1);
    
    % store state and input
    x_history(i) = x_0;
    u_history(i) = u_0;
    i = i + 1;
    

end

%% plot the result

if flag == 1
% if feasible simulation

    figure
    plot([T_0: Ts: T_end], x_ref, 'r');
    hold on;
    plot([T_0: Ts: T_end], x_history, 'b');
    grid on;
    legend('x_ref', 'x');
    xlabel('t');
    ylable('v');
    title("simulation result")


else
    
    ;
    
end

    

end

