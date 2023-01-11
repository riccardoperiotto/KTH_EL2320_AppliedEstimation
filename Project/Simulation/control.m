% Control law for the steering angle of the drones according to their
% status and the targets positions.
% - drone_position: location coordinates for drone location
% - drone_state: define wheter the drone is free or has an apple
% - apples_map: array of apples that the drone is allowed to take 
% - pos_t: spatial coordinates of the tractor
function  [new_status_d, new_theta_d, new_velocity_d] = control(status_d, pos_t, pose_d, y_free)
    global apples_map
    global detect_threshold
    global interact_threshold
    global velocity_d_max
    global velocity_d_std
    
    if status_d == 0
        % drone is free, check if it can reach any apple
        % - distance < detect_threshold
        % - if more apples inside detect_threshold, take the one with lower x coordinate
        % - the drone can take only the apples on its part
        new_status_d = 0;
        i = 1;
        while i <= size(apples_map,2) && new_status_d == 0
            delta_x = apples_map(1, i) - pose_d(1);
            delta_y = apples_map(2, i) - pose_d(2);
            if delta_x^2 +  delta_y^2 <= detect_threshold && apples_map(3,i) == 0 && sign(apples_map(2,i)) == sign(y_free) 
                % Apple detected, change attitude and velocity toward the
                % new target
                new_status_d = i;
                new_theta_d = atan2(delta_y, delta_x);
                new_velocity_d = velocity_d_max;

                apples_map(3,i) = 1;
            else  % TODO: maintain coordinate +-1 w.r.t. tractor
                delta = pos_t + [0;y_free] - pose_d(1:2);
                new_status_d = 0;
                new_theta_d = atan2(delta(2),delta(1));
                new_velocity_d = velocity_d_std;
            end
            i = i+1;
        end
    elseif status_d > 0
        % Drone already targeting an apple, check if apple is reached
        i = status_d;
        delta_x = apples_map(1, i) - pose_d(1);
        delta_y = apples_map(2, i) - pose_d(2);
        % If apple is within the inteact_threshold of the drone, then the
        % apple can be picked
        if delta_x^2 +  delta_y^2 <= interact_threshold
            % Apple caught! Time to go back to the tractor
            new_status_d = -i; 
            new_theta_d = 0;
            new_velocity_d = 0;

            apples_map(3,i) = 2;
        else
            % no real changes
            new_status_d = status_d;
            new_theta_d = pose_d(3);
            new_velocity_d = velocity_d_max;
        end
    elseif status_d < 0
        % Drone is coming back to the tractor, check if it reached it for
        % leaving the apple in the container
        i = - status_d * 1;
        delta_x = pos_t(1) - pose_d(1);
        delta_y = pos_t(2) - pose_d(2);
        if (delta_x^2 +  delta_y^2) <= interact_threshold
            % leave it! The drone leaves the apple and returns free
            new_status_d = 0; 
            new_theta_d = 0;
            new_velocity_d = velocity_d_std;

            apples_map(1:2,i) = [35+i,-12];  % TODO: check this
            apples_map(3,i) = 3;
        else
            % the drone has to go toward the tractor, which is moving
            new_status_d = status_d;
            new_theta_d = atan2(delta_y, delta_x);
            new_velocity_d = velocity_d_max;

            apples_map(1:2, i) = pose_d(1:2);
        end
    end
end