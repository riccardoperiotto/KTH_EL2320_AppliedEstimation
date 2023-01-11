% This function propagates the ideal dynamics to compute the ground truth
%           S_real(4:9,t-1)             6X1
% Outputs: 
%           S_real(t)                   6X1
function [S_real] = ground_truth_drones(S)
    global velocity_d1
    global velocity_d2
    global delta_t
    
    % noise
    std = 0.01;
    eps_d1 = normrnd(0,std);
    eps_d2 = normrnd(0,std);
    
    % drone 1
    u_d1 = delta_t * [velocity_d1 * cos(S(3)); velocity_d1 * sin(S(3)); 0]  + eps_d1;
    
    % drone 2
    u_d2 = delta_t * [velocity_d2 * cos(S(6)); velocity_d2 * sin(S(6)); 0] + eps_d2;
    
    % update all state
    S_real = S + [u_d1; u_d2];
    
    % adjust angles
    S_real(3) = mod(S_real(3)+pi,2*pi) - pi;
    S_real(6) = mod(S_real(6)+pi,2*pi) - pi;

end