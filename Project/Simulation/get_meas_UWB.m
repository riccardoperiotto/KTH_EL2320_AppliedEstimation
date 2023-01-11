% This function simulate a UWB measurement for the drone pose.
% The measurement is the relative position between the drone and the
% tractor in cartesian coordinates.
% Inputs:
%           pos_t     2x1       cartesian coordinate of tractor
%           pos_d     2x1       cartesian coordinate of drone
% Outputs: 
%           z         2x1       cartesian coordinates of relative position

%% How to define d_uwb_receiver
% The d_uwb_receiver variable represents the distance between the two antennas 
% contained in the UWB receiver on the tractor.
% As described in the patent US11215704, this distance should be at the
% most equal to half the wavelength of the transmitted signal (to allow the
% computation of the relative angle between receiver and transmitter using
% the captured phase offset). 
% signal_frequency (f) = 6.5 GHz (decawave DW1000 IC)
% wavelength (λ) = light_speed (c) / signal_frequency (f) = 0.0461 m
% d_uwb_receiver = wavelength (λ) / 2 = 0.02305 m ~ 0.023 m

%% Get drone relative position
% with respect to the first antenna
% we can always assume this to be in the middle of thre tractor
function z = get_meas_UWB(pos_t, pos_d)
    
    global outliers_percentage
    global d_uwb
    
    outlier_std = 1; % [m]
    normal_std = 0.01; % [m]

    % Based on the percentage of outliers, draw measurements with more or
    % less noise
    if rand <= outliers_percentage/100
        % if outlier: std_dev_range = 1 m, std_dev_bearing = 10 deg
        % eps_a1 = normrnd(0,outlier_std_range);
        % eps_a2 = normrnd(0,outlier_std_range);
        eps_x = normrnd(0,outlier_std);
        eps_y = normrnd(0,outlier_std);

    else
        % if not outlier: std_dev_range = 0.01 m, std_dev_bearing = 1 deg
        % eps_a1 = normrnd(0,normal_std_range);
        % eps_a2 = normrnd(0,normal_std_range);
        eps_x = normrnd(0,normal_std);
        eps_y = normrnd(0,normal_std);
    end
     
    pos_a1 = pos_t;
    pos_a2 = pos_t+[d_uwb;0];

    % here, in the simulation, we calculate also r2 to get the difference p
    % in the real chip, p is directly obtained using difference of phase or
    % other methods, so this passage is not required
    r = sqrt((pos_a1(1)-pos_d(1))^2+(pos_a1(2)-pos_d(2))^2); 
    r2 = sqrt((pos_a2(1)-pos_d(1))^2+(pos_a2(2)-pos_d(2))^2);
    
    p = r-r2;

    x = (d_uwb^2+2*r*p-p^2)/(2*d_uwb);
    y = sign(pos_d(2))*(sqrt(r^2-x^2));
    
    z = [x+eps_x;y+eps_y];
end

% outlier_std_range = 1; % [m]
% normal_std_range = 0.01; % [m]
% outlier_std_bearing = 1; % [rad]
% normal_std_bearing = 0.1; % [rad]
% previously, with Carnot for range and bearing
%  % compute absolute position of the antennas
% pos_a1 = pos_t-[d_uwb/2;0];
% pos_a2 = pos_t+[d_uwb/2;0];
% 
% % get the UWB ranges between the drone and the antennas
% r1 = sqrt((pos_a1(1)-pos_d(1))^2+(pos_a1(2)-pos_d(2))^2); % +eps_a1;
% r2 = sqrt((pos_a2(1)-pos_d(1))^2+(pos_a2(2)-pos_d(2))^2); % +eps_a2;
% 
% % compute real range and bearing between drone and the center of the tractor
% r = sqrt(2*r1^2+2*r2^2-d_uwb^2)/2+eps_r;
% alpha = acos((r1^2-r2^2)/(sqrt(2*r1^2+2*r2^2-d_uwb^2)*d_uwb))+eps_b;
% 
% % there is no distinction between positive and negative y; get the sign
% alpha = sign(pos_d(2))*alpha;
% 
% % get measurement
% z = [r;alpha]; 