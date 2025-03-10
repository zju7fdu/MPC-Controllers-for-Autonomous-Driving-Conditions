Ts = 1/10; % Sample time
car = Car(Ts);
[xs, us] = car.steady_state(120 / 3.6);
sys = car.linearize(xs, us);
[sys_lat, sys_lon] = car.decompose(sys);
% Design MPC controller
H_lon = 5; % Horizon length in seconds
mpc_lon = MpcControl_lon(sys_lon, Ts, H_lon);
% Get control input for longitudinal subsystem
u_lon = mpc_lon.get_u(sys_lon.UserData.xs, sys_lon.UserData.us);