function [v, u, Result] = Simulator_2_8(Np, Nc, lambda, u_range, v_range, a_comfort, x_0, v_0, u_0, v_ref, Ts, Tspand, model_mld, modelc)
%Simulator_2_8:  Simulator to simulate the closed-loop behavior of the system
% Input:
%   Np: prediction horizon
%   Nc: control horizon
%   lambda: relative weight of Jinput in the objective function
%   urange: [umin, umax] min and max input
%   v_range: [vmin, vmax] min and max speed
%   a_comfort: comfortable acceleration limitation
%   v_0: initial v 
%   model: MLD model
%   Ts: sampling time
%   v_ref: reference v
%   Tspand: [T_0, T_end]: starting time and stop time
%   model_mld: MLD model
%   modelc: continuous time model
% Output:

%% some parameters 

v1 = 15;
v2 = 28.8575;
v3 = 30;

% limitation of u and limitation of v
umin = u_range(1);
umax = u_range(2);
vmin = v_range(1);
vmax = v_range(2);

% starting time and stop time
T_0 = Tspand(1);
T_end = Tspand(2);

%% judge d_0 and z_0

if v_0 <= v1
    
    d_0 = [1; 1; 1];
    
elseif v_0 <= v2
    
    d_0 = [0; 1; 1];
    
elseif v_0 <=v3
    
    d_0 = [0; 0; 1];
    
else
    
    d_0 = [0; 0; 0];
    
end

z_0 = d_0 .* [u_0; v_0; u_0];

%% start simulation

x_history = zeros(1, length(T_0: Ts: T_end));
v_history = zeros(1, length(T_0: Ts: T_end));
u_history = zeros(1, length(T_0: Ts: T_end));

i = 1;
x_history(i) = x_0;
v_history(i) = v_0;
u_history(i) = u_0;

for t = T_0: Ts: T_end
    
    if (t + Np * Ts > T_end)
    % if Np future > T_end, 
    % then extend reference with the last element in x_ref
    
        temp_v_ref = v_ref([i: 1: end]);
        temp_v_ref = [temp_v_ref; v_ref(end) * ones(Np - length(temp_v_ref), 1)]; 
        
    else
        
        temp_v_ref = v_ref([i: 1: i + Np - 1]);
        
    end
    
    
    [flag, v, u, vc, uc] = Solution_2_7(Np, Nc, lambda, umax, umin, vmax, vmin, a_comfort,... 
                v_0, u_0, model_mld, Ts, temp_v_ref);
            
    if flag == 1
        
        % update state with continuous model
        [temp_t, temp_v] = ode45(modelc, [0, Ts], [10; vc; uc]);
        
    else
    
        fprintf("no feasible optimal solution at time d% \n", t);
        break;
       
    end
    
    % update state and input 
    v_0 = temp_v(end, 2);
    x_0 = x_0 + v_0 * Ts;
    u_0 = u(1);
    
    % store state and input
    x_history(i) = x_0;
    v_history(i) = v_0;
    u_history(i) = u_0;
    i = i + 1;
    

end

%% plot the result

if flag == 1
% if feasible simulation
    v_diff = v_history' - v_ref;
    temp = circshift(u_history, 1);
    u_diff = u_history - temp;
    
    Result.u_diff = u_diff;
    Result.u_history = u_history;
    Result.v_diff = v_diff;
    Result.v_history = v_history;
    Result.x_history = x_history;
    Result.v_ref = v_ref;
end

end

