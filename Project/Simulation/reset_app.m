% Initialize the app. Draw scenario | Initialize parameters
function reset_app(app)
    cla(app.SimulationAxis);
    title(app.SimulationAxis, 'Simulation Time');

    %% Initialize parameters
    % define global variables
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

    % define the problem setting (global localization or tracking)
    global_localization = 0;
       
    % select either default or random position of the apples
    target_type = 'Random'; % or 'Default'
    n_apples = 15; 
    apples_map = zeros(3,n_apples);

    % outlier percentage 
    outliers_percentage = 0;

    % set default values
    show_measurements = true;
    show_ground_truth = true;
    show_estimation = true;

    % particles filters    
    % variables defined just for simplifying the syntax...
    M_t = 1000;
    M_d = 1000;
    R_t = [0.01^2, 0, 0; 0, 0.01^2, 0; 0, 0, 0.01^2];
    Q_t = [0.1^2, 0; 0, 0.1^2];
    R_d = [0.01^2, 0, 0; 0, 0.01^2, 0; 0, 0, 0.1^2];
    Q_d = [0.01^2, 0; 0, 0.01^2];
    lambda_psi_t = 2;
    lambda_psi_d = 2;
    resampling_mode = 1;
    filter_t  = ParticleFilter('T', M_t, R_t, Q_t, lambda_psi_t, R_t, Q_t, lambda_psi_t, resampling_mode);
    filter_d1 = ParticleFilter('D', M_d, R_d, Q_d, lambda_psi_d, R_d, Q_d, lambda_psi_d, resampling_mode);
    filter_d2 = ParticleFilter('D', M_d, R_d, Q_d, lambda_psi_d, R_d, Q_d, lambda_psi_d, resampling_mode);
   
    % Simulation variables
    lane_length         = 40;       % [m]
    lane_width          = 3;        % [m]
    detect_threshold    = 9;        % [m^2] % 3^2
    interact_threshold  = 0.09;     % [m^2] % 0.3^2
    velocity_t          = 1.43; %2.37;    % [m/s]
    omega_t             = 0;        % [rad/s]
    d_tractor_container = 0;        % [m]
    d_uwb               = 0.022;    % [m]
    velocity_d_std      = 1.43; %2.37;    % [m/s] % set equal to that of the tractor and modified by the control function
    velocity_d_max      = 2*velocity_d_std;        % [m/s]
    velocity_d1         = velocity_d_std;        
    velocity_d2         = velocity_d_std;    
    status_d1           = 0;
    status_d2           = 0;
    started             = 0;     
    running             = 0;
    stop_execution      = 1;
    t                   = 0;    % [s]
    delta_t             = 0.1;  % 5 Hz
    n_timesteps         = round((lane_length / velocity_t) / delta_t);
    timestep            = 1;

    %% Initialize user interface
    % enable buttons, spinners and switches
    app.SnapshotButton.Enable = 'on';
    app.TargetDropDown.Enable = 'on';
    app.StartPauseButton.Text = 'Start';
    app.ParticleSlider_t.Enable = 'on';
    app.ParticleEditField_t.Enable = 'on';
    app.ParticleSlider_d.Enable = 'on';
    app.ParticleEditField_d.Enable = 'on';
    
    % display values in spinners
    app.Spinner_R12_t.Value = sqrt(filter_t.R(1, 1));
    % app.Spinner_R2_t.Value = sqrt(filter_t.R(2, 2));
    app.Spinner_R3_t.Value = round(sqrt(filter_t.R(3, 3)) / (2*pi) * 360);   % not used
    app.Spinner_Q12_t.Value = sqrt(filter_t.Q(1, 1));
    app.Spinner_R12_d.Value = sqrt(filter_d1.R(1, 1));
    app.Spinner_R3_d.Value = round(sqrt(filter_d1.R(3, 3)) / (2*pi) * 360);   % not used
    app.Spinner_Q12_d.Value = sqrt(filter_d1.Q(1, 1));
    app.Spinner_Q2_d.Value = round(sqrt(filter_d1.Q(2, 2)) / (2*pi) * 360); % not used
    
    % display number of particles
    app.ParticleSlider_t.Value = filter_t.M;
    app.ParticleEditField_t.Value = filter_t.M;
    app.ParticleSlider_d.Value = filter_d1.M;
    app.ParticleEditField_d.Value = filter_d1.M;
    
    % set global localization switch
    if global_localization
      app.GlobalTrackingSwitch.Value = 'Global';
    else
      app.GlobalTrackingSwitch.Value = 'Tracking';
    end

    % set target type drop down
    app.TargetDropDown.Enable = "on";
    app.AppleNumberSpinner.Enable = "on";
    if strcmp(target_type,'Default')
      app.TargetDropDown.Value = 'Default';
    else
      app.TargetDropDown.Value = 'Random';
    end
    
    % set outliers percentage spinner
    app.OutlierPercentageSpinner.Value = outliers_percentage;
    
    % set outlier detection switch - Tractor
    if lambda_psi_t > 0
        % outlier detection enabled
        app.OutlierSwitch_t.Value = 'On';
        app.OutlierSpinner_t.Visible = 'On';
        app.OutlierSpinner_t.Value = filter_t.lambda_psi;
    else
        % outlier detection disabled
        app.OutlierSwitch_t.Value = 'Off';
        app.OutlierSpinner_t.Value = filter_t.lambda_psi;
    end

    % set outlier detection switch - Drones
    if lambda_psi_d > 0
        % outlier detection enabled
        app.OutlierSwitch_d.Value = 'On';
        app.OutlierSpinner_d.Visible = 'On';
        app.OutlierSpinner_d.Value = filter_d1.lambda_psi;  % same as filter_d2.lambda_psi
    else
        % outlier detection disabled
        app.OutlierSwitch_d.Value = 'Off';
        app.OutlierSpinner_d.Value = filter_d1.lambda_psi;  % same as filter_d2.lambda_psi
    end
    
    % zero the labels
    app.xField_t.Value = 0;
    app.yField_t.Value = 0;
    app.thetaField_t.Value = 0;
    app.Error_x_Field_t.Value = 0;
    app.Error_y_Field_t.Value = 0;
    app.Error_theta_Field_t.Value = 0;
   
    app.xField_d1.Value = 0;
    app.yField_d1.Value = 0;
    app.thetaField_d1.Value = 0;
    app.Error_x_Field_d1.Value =0;
    app.Error_y_Field_d1.Value = 0;
    app.Error_theta_Field_d1.Value = 0;
   
    app.xField_d2.Value = 0;
    app.yField_d2.Value = 0;
    app.thetaField_d2.Value = 0;
    app.Error_x_Field_d2.Value = 0;
    app.Error_y_Field_d2.Value = 0;
    app.Error_theta_Field_d2.Value = 0;

    app.MeasurementField.Value = 0;
    app.OutlierField.Value = 0;

    %% Initialize simulation mode
    % update switches
    if resampling_mode == 0
        app.OffButton.Value = 1;
    elseif resampling_mode == 1
        app.MultinomialButton.Value = 1;
    else
        app.SystematicButton.Value = 1;
    end
    
    % update check boxes
    app.Measurement_CheckBox.Value = show_measurements;
    app.GroundTruth_CheckBox.Value = show_ground_truth;
    app.Estimation_CheckBox.Value = show_estimation;

    %% Read data from file
    fid = fopen('DataSets/dataset_gt_gps.csv', 'r');
    if fid <= 0
      fprintf('Failed to open simoutput file "%s"\n\n', simulationfile);
      return
    end
    simulation_data = {};
    while 1
        line = fgetl(fid);
        if ~ischar(line)
            break
        end
        simulation_data = [simulation_data(:)' {line}];
    end
    fclose(fid);

end