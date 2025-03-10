classdef NmpcControl_overtake < handle

    properties
        % The NMPC problem
        opti

        % Problem parameters
        x0, ref, x0other

        % Most recent problem solution
        sol

        % The input that you want to apply to the system
        u0

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Add any variables you would like to read to debug here
        % and then store them in the NmpcControl function below.
        % e.g., you could place X here and then add obj.X = X
        % in the NmpcControl function below.
        % 
        % After solving the problem, you can then read these variables 
        % to debug via
        %nmpc.sol.value(nmpc.X)
        

        % 
        X, U
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    end

    methods
        function obj = NmpcControl_overtake(car, H)

            import casadi.*

            N_segs = ceil(H/car.Ts); % Horizon steps
            N = N_segs + 1;          % Last index in 1-based Matlab indexing

            nx = 4;
            nu = 2;

            % Define the NMPC optimization problem
            opti = casadi.Opti();
            
            % Parameters (symbolic)
            obj.x0 = opti.parameter(nx, 1);       % initial state
            obj.ref = opti.parameter(2, 1);       % target y, velocity
            obj.x0other = opti.parameter(nx, 1);  % initial state of other car

            % SET THIS VALUE TO BE YOUR CONTROL INPUT
            obj.u0 = opti.variable(nu, 1);

            X = opti.variable(nx, N); % State trajectory
            U = opti.variable(nu, N-1); % Control input trajectory
            Xo = opti.variable(nx, N);
            
            % Add variables for debugging
            obj.X = X;
            obj.U = U;
            Q = diag([1, 38, 1, 29.5]); % State weights
            R = diag([20, 40]); % Input weights

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

            % Define your problem using the opti object created above

            cost = 0;

            % change this line accordingly
            

            % Initial condition constraint
            opti.subject_to(X(:, 1) == obj.x0);
            opti.subject_to(Xo(:, 1) == obj.x0other);
            

            opti.subject_to( obj.u0 ==  U(:,1));

            % Dynamics constraints
            for k = 1:N-1
                x_next = car.f(X(:, k), U(:, k)); % Predict next state using car dynamics
                % Runge-Kutta integration
                k1 = car.f(X(:, k), U(:, k));
                k2 = car.f(X(:, k) + car.Ts/2 * k1, U(:, k));
                k3 = car.f(X(:, k) + car.Ts/2 * k2, U(:, k));
                k4 = car.f(X(:, k) + car.Ts * k3, U(:, k));
                
                x_next = X(:, k) + car.Ts/6 * (k1 + 2*k2 + 2*k3 + k4);
                opti.subject_to(X(:, k+1) == x_next); % Enforce dynamics constraints


                %Dynamics for the other car

                x_other_next = obj.x0other(1) + obj.x0other(4) * car.Ts * k; % Velocity-constant x position prediction
                opti.subject_to(Xo(1, k+1) == x_other_next); % position update
                opti.subject_to(Xo(2:4, k+1) == obj.x0other(2:4));


            
            
                
                
    
    
    
                % State constraints (adjust as per system limits)
                y_min = -0.5; y_max = 3.5; % Lane constraints
                theta_max = 5 * pi / 180; % Heading angle constraint in radians
                v_min = 0; v_max = 50; % Speed constraints in m/s

                opti.subject_to(y_min <= X(2, k)); % Lower bound on Y position
                opti.subject_to(X(2, k) <= y_max); % Upper bound on Y position
                
                opti.subject_to(-theta_max <= X(3, k)); % Lower bound on Heading angle
                opti.subject_to(X(3, k) <= theta_max); % Upper bound on Heading angle
                
                % opti.subject_to(v_min <= X(4, k)); % Lower bound on Velocity
                % opti.subject_to(X(4, k) <= v_max); % Upper bound on Velocity

            % 
            %     % Input constraints (adjust as per actuator limits)
                delta_max = 30 * pi / 180; % Steering angle constraint in radians
                uT_min = -1; uT_max = 1; % Throttle/braking constraint (normalized)
            % 
                % Steering angle constraints
                opti.subject_to(-delta_max <= U(1, k)); % Lower bound on Steering angle
                opti.subject_to(U(1, k) <= delta_max); % Upper bound on Steering angle
                
                % Throttle/braking constraints
                opti.subject_to(uT_min <= U(2, k)); % Lower bound on Throttle/braking
                opti.subject_to(U(2, k) <= uT_max); % Upper bound on Throttle/braking



                % margin distance
                d_safe = 1.2; 
                
                % ellipsoidal parameters
                %a = 4.3 + d_safe + 0.28; 
                V_diff = X(4,k+1) - X(4,k);
                % if double(V_diff) >= 0.2
                %     a = 4.3 + d_safe + 0.9 * V_diff;
                % else
                %     a = 4.3 + d_safe + V_diff;
                % end

                a = 4.3 + d_safe + 2.5*V_diff;%0.01 * X(4, k); % Increase a at large acceleration
                b = 1.8 + d_safe - 0.07; 
                
                %Ellipsoidal matrix
                H = [1/a^2, 0; 0, 1/b^2];
                

                % Car positions
                p = [X(1, k); X(2, k)]; %[lon lat]
                p_L = [Xo(1, k); Xo(2, k)]; 
                
                % collision avoid constraint
                opti.subject_to((p - p_L)' * H * (p - p_L) >= 1);

            
            
                
                
                
                % Objective function: Stage cost
                
            
                state_error = [X(2, k); X(4, k)] - obj.ref; % Tracking error for position and velocity
                control_effort = U(:, k); % Control input
                cost = cost + state_error' * Q([2, 4], [2, 4]) * state_error + control_effort' * R * control_effort;

                %Mitigate Lateral change
                delta_y = X(2, k+1) - X(2, k);
                % cost = cost + 0.1 * delta_y^2; % penalty on lateral change

            end

            for k = 1:N-2
                delta_u_1 = U(1, k+1) - U(1, k);
                delta_u_2 = U(2, k+1) - U(2, k);
                cost = cost + 0.1 * (delta_u_1') * delta_u_1; % Penalty for overreact of control inputs
                cost = cost + 0.1 * (delta_u_2') * delta_u_2;
            end

            
            % Set the cost function to minimize
            opti.minimize(cost);
            



            % opti.minimize(cost);

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Store the defined problem to solve in get_u
            obj.opti = opti;

            % Setup solver
            options = struct;
            options.ipopt.print_level = 0;
            options.print_time = 0;
            options.expand = true;
            obj.opti.solver('ipopt', options);
            % disp(opti.debug.value(X)); 
            % disp(opti.debug.value(U)); 
        end

        function u = get_u(obj, x0, ref, x0other)

            if nargin < 4
                x0other = zeros(4, 1);
            end

            % Compute solution from x0
            obj.solve(x0(1:4), ref, x0other(1:4));

            u = obj.sol.value(obj.u0);
        end

        function solve(obj, x0, ref, x0other)

            % Pass parameter values
            obj.opti.set_value(obj.x0, x0);
            obj.opti.set_value(obj.ref, ref);
            obj.opti.set_value(obj.x0other, x0other);

            obj.sol = obj.opti.solve();   % actual solve
            
            % Set warm start for next solve
            obj.opti.set_initial(obj.sol.value_variables());
            obj.opti.set_initial(obj.opti.lam_g, obj.sol.value(obj.opti.lam_g));
        end
    end
end
