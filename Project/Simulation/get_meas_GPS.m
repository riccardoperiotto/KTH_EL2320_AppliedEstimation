% This function simulate a GPS measurement for the tractor pose.
% The measurement is of the location (x,y) only.
%           S_real(t-1)                 9x1
% Outputs: 
%           z                           2x1
function z = get_meas_GPS(S)
    
    global outliers_percentage
    
    outlier_std = 10; % [m]
    normal_std = 0.1; % [m]

    % Based on the percentage of outliers, draw measurements with more or
    % less noise
    if rand <= outliers_percentage/100
        % if outlier: std_dev = 10 m
        eps = normrnd(0,outlier_std,2,1);  % (mean,std,sz1,sz2)
    else
        % if not outlier: std_dev = 1 m
        eps = normrnd(0,normal_std,2,1);   % (mean,std,sz1,sz2)
    end
    
    z = S(1:2) + eps;
    
end