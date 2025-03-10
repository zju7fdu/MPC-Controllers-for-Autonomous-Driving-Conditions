function offline_results = tube_MPC_sets(Ts, Vs)

    %system setup
    car = Car(Ts);
    [xs, us] = car.steady_state(Vs);
    sys = car.linearize(xs, us);

    %Decompose system
    [sys_lon, ~] = car.decompose(sys);
    [~, Ad, Bd, ~, ~] = Car.c2d_with_offset(sys_lon, Ts);

    %Define disturbance set W
    W = Bd * Polyhedron([-1; 1], [0.5 - us(2); 0.5 + us(2)]);

    %LQR setup
    Q = diag([10, 500]);
    R = 1;
    Bd = -Bd;
    [K, ~, ~] = dlqr(Ad, Bd, Q, R);
    K = -K;

    % Compute minimum robust invariant set (E)
    E = Polyhedron('lb', [0; 0], 'ub', [0; 0]);
    i = 1;
    A_cl = Ad + Bd * K;

    while true
        E_new = E + (A_cl)^i * W;
        E_new.minHRep();
        if norm((A_cl)^i) < 1e-3
            break;
        end
        E = E_new;
        i = i + 1;
    end

    %Tighten constraints
    x_safe = 6;
    x_min = 5;
    Hx = [-1, 0];

    X = Polyhedron(Hx, [-x_min + x_safe]);
    U = Polyhedron([1; -1], [1; 1]);

    X_tightened = X - E;
    X_tightened.minHRep();
    if ~X_tightened.contains([0; 0])
        error("The tightened state constraints do not contain the origin!");
    end

    U_tightened = U - K * E;
    U_tightened.minHRep();

    %Compute terminal set
    Fx_t = X_tightened.A;
    Fu_t = U_tightened.A;
    fx_t = X_tightened.b;
    fu_t = U_tightened.b;

    X_f = Polyhedron('A', [Fx_t; Fu_t * K], 'B', [fx_t; fu_t]);

    iteration = 0;
    while true
        Xf_prev = X_f;
        pre_set = Polyhedron('A', [Xf_prev.A; Xf_prev.A * A_cl], ...
                             'b', [Xf_prev.b; Xf_prev.b]);
        pre_set.minHRep();
        X_f = intersect(Xf_prev, pre_set);
        X_f.minHRep();

        if X_f == Xf_prev
            break;
        end

        iteration = iteration + 1;
    end

    %Offline results
    offline_results.X_tightened = X_tightened;
    offline_results.U_tightened = U_tightened;
    offline_results.X_f = X_f;
    offline_results.Q = Q;
    offline_results.R = R;
    offline_results.x_safe = x_safe;

    save('tube_mpc_data.mat', 'X_tightened', 'U_tightened', 'X_f', 'Q', 'R', 'x_safe');

    figure;
    subplot(2,1,1);
    plot(E,'color','blue');
    xlabel("State variables");
    ylabel('Bounds');
    title('Minimum Invariant Set \epsilon');
    subplot(2,1,2);
    plot(X_f,'color','yellow');
    xlabel('State Variables');
    ylabel('Bounds');
    title('Terminal Set X_{f}');
end
