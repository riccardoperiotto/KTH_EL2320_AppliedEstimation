% Run Simulation of the MCL localization task.
function start_simulation(app)
    % import global variables
    global simulation_data
    global global_localization
    global target_type
    global n_apples
    global apples_map
    global outliers_percentage
    global show_measurements    % display a laser beam for each measurement
    global show_ground_truth    % display groud truth position
    global show_estimation        % display position according to odometry information
    global filter_t
    global filter_d1
    global filter_d2
    global lane_length        
    global lane_width
    global detect_threshold
    global interact_threshold
    global velocity_t
    global omega_t
    global d_tractor_container
    global d_uwb
    global velocity_d_max
    global velocity_d_std
    global velocity_d1
    global velocity_d2
    global status_d1            % define wheter the drone is free or has a target
    global status_d2
    global started    
    global running    
    global stop_execution       % stop execution flag set by app
    global t                    % global time (in seconds
    global delta_t            
    global n_timesteps
    global timestep
    
    %% Init variables and data structures
    stop_execution = 0;

    % Ground truth
    y_d1_free = 1;
    y_d2_free = -1;
    
    S_real = zeros(9, n_timesteps);
    simulation_data = simulation_data(2:end);
    line = simulation_data{1};
    timestep_data = sscanf(line, '%f,%f,%f,%f,%f,%f'); % eventually '%f,'%f','%f'...' if with commas
    S_real(:,1) =  [timestep_data(3),timestep_data(4),0,0,y_d1_free,0,0,y_d2_free,0];
    timestep = 2;

    % save simulation statistics
    sigmas_t = zeros(9, n_timesteps);
    sigmas_d1 = zeros(9, n_timesteps);
    sigmas_d2 = zeros(9, n_timesteps);
    pose_errors_t = zeros(3, n_timesteps);
    pose_errors_d1 = zeros(3, n_timesteps);
    pose_errors_d2 = zeros(3, n_timesteps);
    ground_truth_plots = gobjects(3 , n_timesteps);
    estimated_plots = gobjects(3 , n_timesteps);
    vehicles_loc = gobjects(3);
    short_term_plots = [];
    particle_plots = [];
    total_measurements = 0; % total number of measurements in simulation
    total_outliers = 0;     % total number of detected outliers
    
    %% Simulated environment
    % Generate targets (apples)
    if strcmp(target_type, 'Default')
        rng(0); 
    end
    % the apples distribution follows these rules
    % - along x: uniform distribution along the lane at a distance of 1m+noise
    % - along y: uniform distribution in the interval [-3,-2]U[2,3] meters 
    % - 0: in its origianl position
    % - 1: targeted
    % - 2: took
    % - 3: in the container
    % - WATCH OUT 4!: we are not doing control, just estimating
    apples_map(1,:) = linspace(2, lane_length-3.5, n_apples) + 0.5*randn(size(zeros(1,n_apples)));
    apples_map(2,:) = (2*(rand(1, n_apples)>.5)-1).*(rand(1,n_apples)+2);
    
    % get dimensions
    margin_x = 2;
    margin_y = 10;

    xmin = 0 - margin_x;
    xmax = lane_length + margin_x;
    ymin = - margin_y;
    ymax = lane_width + margin_y;

    % draw map
    cla(app.SimulationAxis)
    apples_loc = plot(app.SimulationAxis, apples_map(1,:), apples_map(2, :), 'ro', 'MarkerSize', 3, 'LineWidth',3);
    hold (app.SimulationAxis, 'on');
    axis(app.SimulationAxis, [xmin xmax ymin ymax]);

    %% Initialize particles
    M_t = filter_t.M;
    M_d1 = filter_d1.M;
    M_d2 = filter_d2.M;
    
    if global_localization
        % spread of particles accross borders of the map in m
        particle_margin_length = 5;  
        particle_margin_width = 2;
        
        % randomly initialized particles
        S_t = [rand(1, M_t) * (lane_length + 2*particle_margin_length) - particle_margin_length;
               rand(1, M_t) * (lane_width + 2*particle_margin_width) - 0.5*lane_width - particle_margin_width;
               rand(1, M_t) * 2*pi - pi;
               1 / M_t * ones(1, M_t)];
        S_d1 = [rand(1, M_d1) * (lane_length + 2*particle_margin_length) - particle_margin_length;
                rand(1, M_d1) * (lane_width + 2*particle_margin_width) - 0.5*lane_width - particle_margin_width;
                rand(1, M_d1) * 2*pi - pi;
                1 / M_d1 * ones(1, M_d1)];
        S_d2 = [rand(1, M_d2) * (lane_length + 2*particle_margin_length) - particle_margin_length;
                rand(1, M_d2) * (lane_width + 2*particle_margin_width) - 0.5*lane_width - particle_margin_width;
                rand(1, M_d2) * 2*pi - pi;
                1 / M_d2 * ones(1, M_d2)];
    else
        % tracking problem with initial estimate
        start_pose_tractor = [-d_tractor_container;0;0];
        start_pose_drone1 = [-d_tractor_container;1;0];
        start_pose_drone2 = [-d_tractor_container;-1;0];
        S_t = [repmat(start_pose_tractor, 1, M_t) ; 
                   1 / M_t * ones(1, M_t)      ];
        S_d1 = [repmat(start_pose_drone1, 1, M_d1) ; 
                    1 / M_d1 * ones(1, M_d1)       ];
        S_d2 = [repmat(start_pose_drone2, 1, M_d2) ; 
                    1 / M_d2 * ones(1, M_d2)       ];
    end
    
    %% Run simulation
    while timestep <= n_timesteps 
        
        fprintf("Iteration: %d \n", timestep)
        
        % stop execution flag set in app -> stop simulation
        if stop_execution
            break;
        end
        
        % if simulation paused, wait until restart
        if ~running  
            pause(1);
            continue;
        end
        
        t = t + delta_t;
         
        % read external data for the current timestep
        line = simulation_data{timestep};
        timestep_data = sscanf(line, '%f,%f,%f,%f,%f,%f'); % eventually '%f,'%f','%f'...' if with commas

        S_real(1:2,timestep) = timestep_data(3:4);
        S_real(4:9,timestep) = ground_truth_drones(S_real(4:9,timestep-1));

         
        %% Applied Estimation
        % Get the measurement from GPS for the tractor
        if mod(timestep, 5)==0 % s(rand(1) < 0.2)
            z_gps = timestep_data(5:6);
            % fprintf('new measure %f %f \n', z_gps(1), z_gps(2))
            total_measurements = total_measurements + 1;
        else
            z_gps = 0;
        end
        
        % Get the measurement from UWB for both drones
        z_uwb_d1 = get_meas_UWB(S_real(1:2,timestep), S_real(4:5,timestep));
        z_uwb_d2 = get_meas_UWB(S_real(1:2,timestep), S_real(7:8,timestep));
        total_measurements = total_measurements + 2;
        
        % run the particle filters
        fprintf('TRACTOR: \n');
        [S_t, outlier_t] = run(filter_t,S_t,z_gps,S_real(3,timestep), 0, 0, velocity_t);
        %fprintf('mean tractor location %d %d\n', mean(S_t(1:2,:),2));
        [mu_t, sigmas_t(:, timestep)] = get_estimation(S_t);
        sigma_x_t = sigmas_t(1, timestep);
        
        fprintf('DRONE 1: \n');
        [S_d1, outlier_d1] = run(filter_d1,S_d1,z_uwb_d1,S_real(6,timestep),mu_t, sigma_x_t, velocity_d1);
        [mu_d1, sigmas_d1(:, timestep)] = get_estimation(S_d1);
        
        fprintf('DRONE 2: \n');
        [S_d2, outlier_d2] = run(filter_d2,S_d2,z_uwb_d2,S_real(9,timestep),mu_t, sigma_x_t, velocity_d2);
        [mu_d2, sigmas_d2(:, timestep)] = get_estimation(S_d2);
        
        total_outliers = total_outliers + outlier_t + outlier_d1 + outlier_d2;
        
        % compute pose error
        pose_errors_t(:, timestep) = get_errors(S_real(1:3, timestep), mu_t);
        pose_errors_d1(:, timestep) = get_errors(S_real(4:6, timestep), mu_d1);
        pose_errors_d2(:, timestep) = get_errors(S_real(7:9, timestep), mu_d2);
        
        %% Plot Simulation
        delete(vehicles_loc);
        delete(apples_loc);
        
        % plot particles, predicted location and ellipses
        delete(particle_plots);
        particle_plots(1) = scatter(app.SimulationAxis, S_t(1, :), S_t(2, :), 3, 'filled', 'x', 'MarkerEdgeColor', [0.0706,0.2941, 0.3725], 'MarkerFaceColor', [0.1445, 0.5882, 0.7451]);
        particle_plots(2) = scatter(app.SimulationAxis, S_d1(1, :), S_d1(2, :), 3, 'filled', 'x', 'MarkerEdgeColor', [0.5,0,0.5], 'MarkerFaceColor', [0.8,0,0.8]);
        particle_plots(3) = scatter(app.SimulationAxis, S_d2(1, :), S_d2(2, :), 3, 'filled', 'x', 'MarkerEdgeColor', [1,0.8353,0.1882], 'MarkerFaceColor', [1, 0.9451, 0.1882]);
        
        % plot robot location: solid line
        if show_ground_truth
            ground_truth_plots(1, timestep) = plot(app.SimulationAxis, S_real(1,1:timestep), S_real(2,1:timestep), '-b');
            vehicles_loc(1) = rectangle(app.SimulationAxis, 'Position', [S_real(1,timestep)-1 S_real(2,timestep)-0.5 2 1], 'FaceColor', [0,0,1,0.5]);
            ground_truth_plots(2, timestep) = plot(app.SimulationAxis, S_real(4,1:timestep), S_real(5,1:timestep), '-m');
            vehicles_loc(2) = plot(app.SimulationAxis, S_real(4,timestep), S_real(5,timestep), 'o', 'MarkerSize', 6, 'LineWidth',2, 'Color', [1,0,1]); %, 'MarkerEdgeColor',[1,0,1,0.3]);
            ground_truth_plots(3, timestep) = plot(app.SimulationAxis, S_real(7,1:timestep), S_real(8,1:timestep), '-', 'Color', [0.93,0.69,0.13]);
            vehicles_loc(3) = plot(app.SimulationAxis, S_real(7,timestep), S_real(8,timestep), 'o', 'MarkerSize', 6, 'LineWidth',2,'Color', [0.93,0.69,0.13]);% , 'MarkerEdgeColor',[1,1,0,0.3]);
        else
            delete(ground_truth_plots(1:3, 1:timestep))
        end

        % plot robot estimated location: x for every estimated location
        if show_estimation
            estimated_plots(1, timestep) = plot(app.SimulationAxis, mu_t(1), mu_t(2), 'xb');
            estimated_plots(2, timestep) = plot(app.SimulationAxis, mu_d1(1), mu_d1(2), 'xm');
            estimated_plots(3, timestep) = plot(app.SimulationAxis, mu_d2(1), mu_d2(2), 'x','MarkerEdgeColor', [0.93,0.69,0.13],'MarkerFaceColor',[0.93,0.69,0.13]);
        else
            delete(estimated_plots(1:3, 1:timestep))
        end
        
        % display simulation time
        title(app.SimulationAxis, sprintf('Simulation Time: %.1f s', round(t, 1)));
        
        % delete short-term-plots
        for k = 1:length(short_term_plots)
            if k < 5
                delete(short_term_plots(k))
            elseif size(z_gps,1) > 1 && k == 5
                delete(short_term_plots(k))
            end
        end
        
        % Measurements are displayed from the ground truth object due to
        % inaccurate state estimate in case of several hypotheses.
        % A measurement is considered incorrectly associated if the most
        % frequent association among all particles is incorrect.
        if show_measurements
            % plot measurements
            plot_colors = ['g', 'r']; % correctly associated measurements: green | outliers: yellow
            true_pose = S_real(:,timestep);
            
            % plot uwb measurement drone 1
            plot_color = plot_colors(outlier_d1+1);
            % measurement_endpoint_d1 = true_pose(4:5) - [z_uwb_d1(1,1)*cos(z_uwb_d1(2,1));z_uwb_d1(1,1)*sin(z_uwb_d1(2,1))];
            measurement_endpoint_d1 = true_pose(1:2) + z_uwb_d1;
            short_term_plots(1) = plot(app.SimulationAxis, measurement_endpoint_d1(1), measurement_endpoint_d1(2), strcat(plot_color,'.'));
            short_term_plots(2) = plot(app.SimulationAxis, [true_pose(1), measurement_endpoint_d1(1)], [true_pose(2), measurement_endpoint_d1(2)], plot_color);
            
            % plot uwb measurement drone 2
            plot_color = plot_colors(outlier_d2+1);
            % measurement_endpoint_d2 = true_pose(7:8) - [z_uwb_d2(1,1)*cos(z_uwb_d2(2,1));z_uwb_d2(1,1)*sin(z_uwb_d2(2,1))];
            measurement_endpoint_d2 = true_pose(1:2) + z_uwb_d2;
            short_term_plots(3) = plot(app.SimulationAxis, measurement_endpoint_d2(1), measurement_endpoint_d2(2), strcat(plot_color,'.'));
            short_term_plots(4) = plot(app.SimulationAxis, [true_pose(1), measurement_endpoint_d2(1)], [true_pose(2), measurement_endpoint_d2(2)], plot_color);
            
            % plot gps measurement tractor
            if size(z_gps, 1) > 1
                plot_color = plot_colors(outlier_t+1);
                short_term_plots(5) = plot(app.SimulationAxis, z_gps(1,1), z_gps(2,1), strcat(plot_color,'.'));
            end
        end
         

        % update apples positons
        apples_loc = plot(app.SimulationAxis, apples_map(1,:), apples_map(2, :), 'ro', 'MarkerSize', 2, 'LineWidth',2);
        hold (app.SimulationAxis, 'on');
        axis(app.SimulationAxis, [xmin xmax ymin ymax]);
        
        %% Control
        % modify drones velocitioes and steering angles according to their
        % positions and the targets (apples) locations
        % status can be:
        %  0: free
        %  i: reaching the target at index i
        % -1: reaching back the tractor
        [status_d1, S_real(6, timestep), velocity_d1] = control(status_d1, S_real(1:2, timestep), S_real(4:6, timestep), y_d1_free);
        [status_d2, S_real(9, timestep), velocity_d2] = control(status_d2, S_real(1:2, timestep), S_real(7:9, timestep), y_d2_free);
        

        %% Update labels
        % update fields for estimated pose and error
        app.xField_t.Value = mu_t(1);
        app.yField_t.Value = mu_t(2);
        app.thetaField_t.Value = round(mu_t(3) / (2*pi) * 360);
        app.Error_x_Field_t.Value = pose_errors_t(1, timestep);
        app.Error_y_Field_t.Value = pose_errors_t(2, timestep);
        app.Error_theta_Field_t.Value = round(pose_errors_t(3, timestep) / (2*pi) * 360);
        
        app.xField_d1.Value = mu_d1(1);
        app.yField_d1.Value = mu_d1(2);
        app.thetaField_d1.Value = round(mu_d1(3) / (2*pi) * 360);
        app.Error_x_Field_d1.Value = pose_errors_d1(1, timestep);
        app.Error_y_Field_d1.Value = pose_errors_d1(2, timestep);
        app.Error_theta_Field_d1.Value = round(pose_errors_d1(3, timestep) / (2*pi) * 360);
        
        app.xField_d2.Value = mu_d2(1);
        app.yField_d2.Value = mu_d2(2);
        app.thetaField_d2.Value = round(mu_d2(3) / (2*pi) * 360);
        app.Error_x_Field_d2.Value = pose_errors_d2(1, timestep);
        app.Error_y_Field_d2.Value = pose_errors_d2(2, timestep);
        app.Error_theta_Field_d2.Value = round(pose_errors_d2(3, timestep) / (2*pi) * 360);
        
        app.MeasurementField.Value = total_measurements;
        app.OutlierField.Value = total_outliers;
        
        %% rest
        timestep = timestep + 1;
        pause(0.2);
    end
     
     for k = 1:length(short_term_plots)
            if k < 5
                delete(short_term_plots(k))
            elseif size(z_gps,1) > 1 && k == 5
                delete(short_term_plots(k))
            end
     end

    if stop_execution == 0
        % get error statistics
        mex_t = mean(pose_errors_t(1,:));
        mey_t = mean(pose_errors_t(2,:));
        mex_d1 = mean(pose_errors_d1(1,:));
        mey_d1 = mean(pose_errors_d1(2,:));
        mex_d2 = mean(pose_errors_d2(1,:));
        mey_d2 = mean(pose_errors_d2(2,:));

        maex_t = mean(abs(pose_errors_t(1,:)));
        maey_t = mean(abs(pose_errors_t(2,:)));
        maex_d1 = mean(abs(pose_errors_d1(1,:)));
        maey_d1 = mean(abs(pose_errors_d1(2,:)));
        maex_d2 = mean(abs(pose_errors_d2(1,:)));
        maey_d2 = mean(abs(pose_errors_d2(2,:)));
 

        % plot errors and covariance
        figure('Name', 'Evolution State Estimation Errors');
        clf;
        subplot(3,2,1);
        plot(pose_errors_t(1,:));
        xlim([0 n_timesteps]);
        ylabel('error\_x [m]');
        title(sprintf('Tractor: MAE_x = %.2fm', maex_t));
        subplot(3,2,2);
        plot(pose_errors_t(2,:));
        xlim([0 n_timesteps]);
        ylabel('error\_y [m]');
        title(sprintf('Tractor: MAE_y = %.2fm', maey_t));
        subplot(3,2,3);
        plot(pose_errors_d1(1,:));
        xlim([0 n_timesteps]);
        ylabel('error\_x [m]');
        title(sprintf('Drone 1: MAE_x = %.2fm', maex_d1));
        subplot(3,2,4);
        plot(pose_errors_d1(2,:));
        xlim([0 n_timesteps]);
        ylabel('error\_y [m]');
        title(sprintf('Drone 1: MAE_y = %.2fm', maey_d1));
        subplot(3,2,5);
        plot(pose_errors_d2(1,:));
        xlim([0 n_timesteps]);
        ylabel('error\_x [m]');
        title(sprintf('Drone 2: MAE_x = %.2fm', maex_d2));
        subplot(3,2,6);
        plot(pose_errors_d2(2,:));
        xlim([0 n_timesteps]);
        ylabel('error\_y [m]');
        title(sprintf('Drone 2: MAE_y = %.2fm', maey_d2));
        

        figure('Name', 'Evolution State Estimation Covariance Matrix');
        clf;
        subplot(3,2,1);
        plot(sqrt(sigmas_t(1,:)));
        xlim([0 n_timesteps]);
        ylabel('uncertainy\_x [m]');
        title('Tractor \sigma_x');
        subplot(3,2,2);
        plot(sqrt(sigmas_t(5,:)));
        xlim([0 n_timesteps]);
        ylabel('uncertainy\_y [m]');
        title('Tractor \sigma_y');
        subplot(3,2,3);
        plot(sqrt(sigmas_d1(1,:)));
        xlim([0 n_timesteps]);
        ylabel('uncertainty\_x [m]');
        title('Drone 1 \sigma_x');
        subplot(3,2,4);
        plot(sqrt(sigmas_d1(5,:)));
        xlim([0 n_timesteps]);
        ylabel('uncertainty\_y [m]');
        title('Drone 1 \sigma_y');
        subplot(3,2,5);
        plot(sqrt(sigmas_d2(1,:)));
        xlim([0 n_timesteps]);
        ylabel('uncertainty\_x [m]');
        title('Drone 2 \sigma_x');
        subplot(3,2,6);
        plot(sqrt(sigmas_d2(5,:)));
        xlim([0 n_timesteps]);
        ylabel('uncertainty\_y [m]');
        title('Drone 2 \sigma_y');        
    end

end

% returns statistic given a set of particles
function [mu, sigma] = get_estimation(S)
    mu = mean(S(1:3,:), 2);
    mu(3) = atan2(mean(sin(S(3, :))), mean(cos(S(3, :))));
    pos_sigma = cov(S(1,:),S(2,:));
    var_theta = var(sin(S(3, :))) + var(cos(S(3, :)));
    sigma = zeros(3,3);
    sigma(1:2, 1:2) = pos_sigma;
    sigma(3, 3) = var_theta;
    sigma = reshape(sigma,9,1);
end

% returns statistic given a set of particles
function pose_error = get_errors(S, mu)
    pose_error = S - mu;
    pose_error(3) = mod(pose_error(3)+pi,2*pi)-pi;
end