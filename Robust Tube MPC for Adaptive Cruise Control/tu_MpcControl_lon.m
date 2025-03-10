classdef tu_MpcControl_lon < MpcControlBase

    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   x0           - initial state (estimate)
            %   V_ref, u_ref - reference state/input
            %   d_est        - disturbance estimate
            %   x0other      - initial state of other car
            % OUTPUTS
            %   u0           - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            N_segs = ceil(mpc.H/mpc.Ts); % Horizon steps
            N = N_segs + 1;              % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);

            % Targets
            V_ref = sdpvar(1);
            u_ref = sdpvar(1);

            % Disturbance estimate (Ignore this before Todo 4.1)
            d_est = sdpvar(1);

            % Initial states
            x0 = sdpvar(nx, 1);
            x0other = sdpvar(nx, 1); % (Ignore this before Todo 5.1)

            % Input to apply to the system
            u0 = sdpvar(nu, 1);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D
            %       are the DISCRETE-TIME MODEL of your system.
            %       You can find the linearization steady-state
            %       in mpc.xs and mpc.us.

            %
            load('tube_mpc_data.mat', 'X_tightened', 'U_tightened', 'X_f','Q','R', 'x_safe');

            X_tightened_a = X_tightened.A;
            G_tightened_a = U_tightened.A;
            X_tightened_b = X_tightened.b;
            g_tightened_a = U_tightened.b;
            x_safe = [x_safe;0];

            

            % Define prediction horizon variables
            delta = sdpvar(nx, N); % States over the horizon
            V = sdpvar(nu, N-1); % Inputs over the horizon

            A = mpc.A;
            B = mpc.B;
            % C = mpc.C;
            % D = mpc.D;    

            % Feedback gain to compute input
            [K,~,~] = dlqr(A,B,Q,R);

            K = -K;

            [P,~,~] = dare(A,B,Q,R);



            % Replace this line and set u0 to be the input that you
            % want applied to the system. Note that u0 is applied directly
            % to the nonlinear system. You need to take care of any 
            % offsets resulting from the linearization.
            % If you want to use the delta formulation make sure to
            % substract mpc.xs/mpc.us accordingly.
         
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            obj = delta(:,N)'*P*delta(:,N);
            con = [];

            con = con + (delta(:,1) == x0other - x_safe - x0);

            for i = 1:N-1
                con = [con, delta(:, i+1) == A* (delta(:, i)) - B*(V(:,i)) +B * d_est  ];  
                con = [con, G_tightened_a*(V(:,i)) <= g_tightened_a];
                con = [con, X_tightened_a*(delta(:,i)) <= X_tightened_b];
                obj = obj + delta(:,i)'*Q*delta(:,i) + V(:,i)'*R*V(:,i);
            end
            con = con + (X_f.A*delta(:,N) <= X_f.b);
            con = con + (u0 == K*(x0other - x_safe - x0 - delta(:,1)) + V(:,1));





            % Pass here YALMIP sdpvars which you want to debug. You can
            % then access them when calling your mpc controller like

            debugVars = {};

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {x0, V_ref, u_ref, d_est, x0other}, {u0, debugVars{:}});
        end

        % Computes the steady state target which is passed to the
        % controller
        function [Vs_ref, us_ref] = compute_steady_state_target(mpc, ref, d_est)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            %   d_est  - disturbance estimate (Ignore before Todo 4.1)
            % OUTPUTS
            %   Vs_ref, us_ref - steady-state target
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
            d_u = ((eye(2)-A)*(Vs_ref-xs(2)) -B*d_est)./B;
            us_ref = d_u(2) + us;
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end