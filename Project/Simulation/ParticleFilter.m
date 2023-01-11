% This file declares and defines the class ParticleFilter.
classdef ParticleFilter
    properties
        entity_type % define wheter the filter is for the tractor or for the drones
        M           % number of particles     
        R           % covariance matrix of the motion model
        Q           % covariance matrix of the measurement model - TRUCK
        lambda_psi  % threshold on average likelihood for outlier detection

        % default parameters for chosen dataset for reset button in app
        default_R
        default_Q
        default_lambda_psi % threshold on average likelihood for outlier detection
        
        % resample type
        resampling_mode         
    end
    methods
        % Constructor -----------------------------------------------------
        function obj = ParticleFilter(entity_type, M, R, Q, lambda_psi, default_R, default_Q, default_lambda_psi,resampling_mode)
            if nargin == 9
                obj.entity_type = entity_type;
                obj.M = M;
                obj.R = R;
                obj.Q = Q;
                obj.lambda_psi = lambda_psi;
                obj.default_R = default_R;
                obj.default_Q = default_Q;
                obj.default_lambda_psi = default_lambda_psi;
                obj.resampling_mode = resampling_mode;
            end
        end
        % Reset app -------------------------------------------------------
        function obj = reset(obj)
            obj.R = obj.default_R;
            obj.Q = obj.default_Q;
            obj.lambda_psi = obj.default_lambda_psi;
        end
        
        % Filter Prediction Step ------------------------------------------
        function S_bar = predict(obj, S, theta, v)
            global delta_t
            S(3,:) = theta;
            u = delta_t * [v * cos(S(3,:)); v * sin(S(3,:)); zeros(1,obj.M); zeros(1,obj.M)];
            S_bar = S + u + [mvnrnd([0;0;0],obj.R,obj.M)'; zeros(1,obj.M)];
            S_bar(3, :) = mod(S_bar(3,:)+pi,2*pi) - pi;
        end
        
        % Observation Model -----------------------------------------------
        function z_hat = observation_model(obj,S_bar,mu_t)
            % observation function
            if strcmp(obj.entity_type,'T')
                % tractor's observation model
                h = [ S_bar(1,:) ;
                      S_bar(2,:) ];
            else
                % adjust bearing interval
                S_bar(3,:) = mod( S_bar(3,:) + pi, 2 * pi) - pi;
                
                % drone's observation model
                h = [S_bar(1,:)-mu_t(1); S_bar(2,:)-mu_t(2)];
                
            end
            
            % observation
            z_hat = h; 
        end
        
        % Check outliers (association) ------------------------------------
        function [outlier,psi] = check_outliers(obj,z,z_hat)
            % compute innovation
            nu = z - z_hat;
                          
            % compute likelihood
            psi = 1/(2*pi*sqrt(det(obj.Q))) * exp( -1/2 * sum(nu .* (obj.Q\ nu),1));
            
            % Outlier detection
            outlier = mean(psi) < obj.lambda_psi; 
            
        end
        
        % Update weights --------------------------------------------------
        function S_bar = weight(obj, S_bar,outlier,psi)
            if outlier
                psi(1,:) = 1;
            end
            S_bar(4,:) = psi;
            S_bar(4,:) = S_bar(4,:)/sum(S_bar(4,:));
            if isnan(S_bar(4,4))
                b = 3;
            end
        end
        
        % Multinomial Resampling ------------------------------------------
        function S = multinomial_resample(obj, S_bar)
            cdf = cumsum(S_bar(4,:));
            S = zeros(4,obj.M);
            for m = 1 : obj.M
                r_m = rand;
                i = find(cdf >= r_m,1,'first');
                % S(1:3,m) = S_bar(1:3,i);   % alternatively, try lines below
                try
                    S(1:3,m) = S_bar(1:3,i);
                catch ME
                    a = 2;
                end
            end
            S(4,:) = 1/obj.M*ones(1,obj.M);  
        end
        
        % Systematic Resampling -------------------------------------------
        function S = systematic_resample(obj, S_bar)
            cdf = cumsum(S_bar(4,:));
            S = zeros(4,obj.M);
            r_0 = rand / obj.M;
            for m = 1 : obj.M        
                i = find(cdf >= r_0,1,'first');
                S(1:3,m) = S_bar(1:3,i);
                r_0 = r_0 + 1/obj.M;
            end
            S(4,:) = 1/obj.M*ones(1,obj.M);
        end
        
        %  the Filter --------------------------------------------------
        function [S,outlier] = run(obj, S, z, theta, mu_t, sigma_x_t, v)
            % prediction
            S_bar = predict(obj,S,theta,v);
             
            % the measurement is not the "null scalar" we set 
            % in case of no measurement
            if size(z,1) > 1 && sigma_x_t < 100
                fprintf('update\n')
                % observation model
                z_hat = observation_model(obj,S_bar,mu_t);

                % check outlier
                [outlier,psi] = check_outliers(obj,z,z_hat);

                % update weights
                S_bar = weight(obj,S_bar,outlier,psi);

                % resample particles
                switch obj.resampling_mode
                    case 0
                        S = S_bar;
                    case 1
                        S = multinomial_resample(obj,S_bar);
                    case 2
                        S = systematic_resample(obj,S_bar);
                end 
            else
              S = S_bar; 
              outlier = 0;
            end
        end
        
    end
    
end

