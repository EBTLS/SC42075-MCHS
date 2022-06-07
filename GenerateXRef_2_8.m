function v_ref = GenerateXRef_2_8(Ts, alpha)
%GenerateXRef_2_8 Generate the Corresponding x_ref Signal
% Input:
%   Ts: sampling time
%   alpha: parameter of PWA model
% Output:
%   v_ref: reference v

% constant
T_0 = 0; T_end = 25;
T1 = 3; T2 = 9; T3 = 15; T4 = 18; T5 = 21;
    
v_ref = 5 * ones(length(T_0: Ts: T_end), 1);

% 0 <= t <= 3
temp = 0.85 * alpha * ones(length(T_0: Ts: T1), 1);
v_ref = temp;

% 3 < t <= 9
temp = 1.2 * alpha * ones((length(T1: Ts: T2)-1), 1);
v_ref = [v_ref; temp];

% 9 < t <= 15
t = T2 + Ts;
for i = 1 : (length(T2: Ts: T3)-1)
    temp = 1.2 * alpha - 1/12 * alpha * (t - 9);
    t = t + Ts;
    v_ref = [v_ref; temp];
end

% 15 < t <= 18
temp = 0.7 * alpha * ones((length(T3: Ts: T4)-1), 1);
v_ref = [v_ref; temp];

% 18 < t <= 21
t = T4 + Ts;
for i = 1 : (length(T4: Ts: T5)-1)
    temp = 0.7 * alpha + 4/15 * alpha * (t - 18);
    t = t + Ts;
    v_ref = [v_ref; temp];
end

% 21 < t <= 25
temp = 0.9 * alpha * ones((length(T5: Ts: T_end)-1), 1);
v_ref = [v_ref; temp];


plot([T_0: Ts: T_end], v_ref, '*');
grid on;
legend('v_{ref}');
xlabel('t');
ylabel('v_{ref}');
title("speed reference")
end

