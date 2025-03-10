function terminal_set = lat_terminal(mpc_lat)
    A = mpc_lat.A;
    B = mpc_lat.B;
    us = mpc_lat.us;
    Q = eye(2);
    R = 10;
    [K,~,~] = dlqr(A, B, Q, R);
    K = -K;
    
    A_cl = A + B*K;
    
    %construct constraints for X and U
    Hu = [1;-1];
    hu = [1;1];

    Hx = [1,0;0,-1;-1,0;0,1];
    hx = [3.5;0.0873;0.5;0.0873];
    
    HH = [Hx;Hu*K];
    hh = [hx;hu];
    omega = Polyhedron(HH,hh);
    
    i = 1;

    % omega = X_tightened;
    while true
        X_f_prev = omega; % Start with tightened state constraints


        
        F = X_f_prev.A;
        f = X_f_prev.b;

        
    
        FF = [F;F * A_cl];
        
        ff = [f;f];
        

    
        omega = Polyhedron(FF,ff);

        % omega = intersect(X_f_prev, projection(Pre_Omega, 1 : size(A, 2)));
        if omega == X_f_prev
            break;
        end

        % figure;
        % plot(omega, 'Color', [1, 1 - i * 0.1, i * 0.1]); % Dynamic color change for visualization
	    % fprintf('Iteration %i... not yet equal\n', i)
	    % pause

	    i = i + 1;
    end
    terminal_set.X_f = omega; % Final terminal set
    terminal_set.X= Polyhedron(Hx,hx);
    figure;
    subplot(2,1,1);
    % terminal_set.plot();
    plot(terminal_set.X_f, 'Color', 'blue');
    subplot(2,1,2);
    plot(Polyhedron(Hx,hx),'color','yellow');
    title('Terminal Set for Lateral System');
    xlabel('State Variables');
    ylabel('Bounds');

   
    
end