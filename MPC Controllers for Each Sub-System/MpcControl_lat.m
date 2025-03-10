classdef MpcControl_lat < MpcControlBase 
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   x0           - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   u0           - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(mpc.H / mpc.Ts); % Horizon steps
            N = N_segs + 1;               % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets
            x_ref = sdpvar(nx, 1);        % Reference state
            u_ref = sdpvar(nu, 1);        % Reference input

            % Initial states
            x0 = sdpvar(nx, 1);           % Initial state
            x0other = sdpvar(nx, 1);      % Not used

            % Input to apply to the system
            u0 = sdpvar(nu, 1);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % Define state and input variables over the horizon
            X = sdpvar(nx, N);      % State variables
            U = sdpvar(nu, N-1);    % Input variables

            % Define cost and constraints
            obj = 0;
            con = [];

            % Dynamics, constraints, and objective function
            A = mpc.A;
            B = mpc.B;
            xs = mpc.xs;
            us = mpc.us;

            % Cost matrices
            Q = 100* eye(nx);            % State penalty
            R = 1;                    % Input penalty

            % Initial state constraint
            con = con + (x0 == X(:, 1));

            % Dynamics, input, and state constraints
            for k = 1:N-1
                % Dynamics constraints
                con = con + ((X(:, k+1) - xs) == A * (X(:, k) - xs) + B * (U(:, k) - us));

                % State bounds
                con = con + (-0.5 <= X(1, k)) + ( X(1, k)<= 3.5);      % y bounds
                con = con + (-0.0873 <= X(2, k)) + (X(2, k) <= 0.0873); % θ bounds

                % Input bounds
                con = con + (-0.5236 <= U(:, k)) + (U(:, k)<= 0.5236); % δ bounds

                % Objective function
                obj = obj + (X(:, k) - x_ref)' * Q * (X(:, k) - x_ref) + ...
                      (U(:, k) - u_ref)' * R * (U(:, k) - u_ref);
            end

            % Terminal cost
            obj = obj + (X(:, N) - x_ref)' * Q * (X(:, N) - x_ref);

            % Define u0 as the first step input
            con = con + (u0 == U(:, 1));

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return optimizer
            ctrl_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ...
                {x0, x_ref, u_ref, x0other}, {u0});
        end
        
        % Computes the steady state target which is passed to the
        % controller
        function [xs_ref, us_ref] = compute_steady_state_target(mpc, ref)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs_ref, us_ref - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Steady-state system
            A = mpc.A;
            B = mpc.B;

            % Linearization steady-state
            xs = mpc.xs;
            us = mpc.us;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            xs_ref = [ref(1);0];
            us_ref = 0;
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end

      