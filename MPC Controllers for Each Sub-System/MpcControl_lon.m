classdef MpcControl_lon < MpcControlBase

    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   x0           - initial state (estimate)
            %   V_ref, u_ref - reference velocity/input
            %   d_est        - disturbance estimate
            %   x0other      - initial state of other car
            % OUTPUTS
            %   u0           - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % invariant_properties = compute_sigma_set(mpc);
            % tube_params = tube_mpc_set(invariant_properties);


            N_segs = ceil(mpc.H / mpc.Ts); % Horizon steps
            N = N_segs + 1;               % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);

            % Targets
            V_ref = sdpvar(1);            % Reference velocity
            u_ref = sdpvar(1);            % Reference input

            % Disturbance estimate (not used yet)
            d_est = sdpvar(1);

            % Initial states
            x0 = sdpvar(nx, 1);           % Initial state
            x0other = sdpvar(nx, 1);      % Not used

            % Input to apply to the system
            u0 = sdpvar(nu, 1);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

            % Define state and input variables over the horizon
            X = sdpvar(1, N);      % State variables
            U = sdpvar(1, N-1);    % Input variables

            % Define cost and constraints
            obj = 0;
            con = [];

            xs = mpc.xs(2);                  % Linearized steady-state state
            us = mpc.us;                  % Linearized steady-state input
            % [Vs_ref, us_ref] = compute_steady_state_target(mpc, ref, d_est);


            % Dynamics, constraints, and objective function
            A = mpc.A(2,2);
            B = mpc.B(2,1);

            % Cost matrices
            Q = 10;  % Penalty on velocity deviation
            R = 1;        % Penalty on input

            
            %Terminal Components (from tube_mpc_sets)
            % [C_infinity, Vf] = tube_mpc_sets.terminal_sets(mpc.params); 

            % Initial state constraint
            con = con + (X(1) == x0(2));

            % Define u0 as the first step input
            con = con + (U(:,1) == u0);

            % Dynamics, input, and state constraints
            for k = 1:N-1
                % Dynamics constraints
                con = con + ((X(k+1)) == A * (X(k)) + B * (U(:, k) + d_est));


                % Input bounds: acceleration constraint
                con = con + ((-1 <= U(:, k)) & ( U(:, k) <= 1)); % Input limits (e.g., acceleration in m/s^2)

                

                
                % Objective function: minimize deviation from reference
                obj = obj + (X(k) - V_ref)' * Q * (X(k) - V_ref) + ...
                      (U(:, k) - u_ref)' * R * (U(:, k) - u_ref);
            end
            

            % Terminal cost
            obj = obj + (X(N) - V_ref)' * Q * (X(N) - V_ref);

            % Terminal Constraint
            % con = con + (C_infinity.contains(X(:, N)));
        
            % Terminal Costs
            % obj = obj + Vf(X(:, N));



            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return optimizer
            ctrl_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ...
                {x0, V_ref, u_ref, d_est, x0other}, {u0});

        end
        
        % Computes the steady state target which is passed to the controller
        function [Vs_ref, us_ref] = compute_steady_state_target(mpc, ref, d_est)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            %   d_est  - disturbance estimate (not used yet)
            % OUTPUTS
            %   xs_ref, us_ref - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Steady-state subsystem
            A = mpc.A;
            B = mpc.B;

            % Subsystem linearization steady-state
            xs = mpc.xs;
            us = mpc.us;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            Vs_ref = ref;
            delta_u = ((eye(2)-A)*(Vs_ref-xs(2)) -B*d_est)./B;
            us_ref = delta_u(2) + us;


            % A = mpc.A(2, 2);
            % B = mpc.B(2, 1);
            % 
            % % Subsystem linearization steady-state
            % xs = mpc.xs(2);
            % us = mpc.us;
            % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % 
            % % Reference state: V = ref(1)
            % 
            % Vs_ref=ref; % Velocity reference
            % us_ref = (1-A)/B*(Vs_ref-xs)+us - d_est;     % Input reference (acceleration)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
