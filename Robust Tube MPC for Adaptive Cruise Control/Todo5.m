
Ts=1/10;

ref = [0 120/3.6]'; % (y ref, V ref)

car = Car(Ts);
[xs, us] = car.steady_state(120 / 3.6);
sys = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys);
% Design MPC controller
H_lon =5; % Horizon length in seconds
mpc_lon = MpcControl_lon(sys_lon, Ts, H_lon);
mpc_lat = MpcControl_lat(sys_lat, Ts, H_lon);
results = tube_MPC_sets(Ts, ref(2));





params.myCar.model = car;
otherRef = 100 / 3.6;
H_lon = 3;

mpc_tube = tu_MpcControl_lon(sys_lon, Ts, H_lon);
mpc_lat = MpcControl_lat(sys_lat, Ts, H_lon);
mpc = car.merge_lin_controllers(mpc_tube, mpc_lat);




params = {};
params.Tf = 25;
params.myCar.model = car;
params.myCar.x0 = [0 0 0 100/3.6]';
params.myCar.u = @mpc.get_u;
params.myCar.ref = ref;
params.otherCar.model = car;
params.otherCar.x0 = [15 0 0 100/3.6]';
params.otherCar.u = car.u_const(otherRef);
% params.otherCar.u = car.u_fwd_ref();
% params.otherCar.ref = car.ref_robust();
result = simulate(params);
visualization(car, result);



% mpc = car.merge_lin_controllers(mpc_tube, mpc_lat);
% x0 = [0 0 0 80/3.6]'; % (x, y, theta, V)
% ref1 = [0 80/3.6]'; % (y ref, V ref)
% ref2 = [3 120/3.6]'; % (y ref, V ref)
% params = {};
% params.Tf = 25;
% params.myCar.x0 = [0 0 0 115/3.6]';
% params.myCar.u = @mpc.get_u;
% params.myCar.ref = ref;
% params.otherCar.model = car;
% params.otherCar.x0 = [8 0 0 120/3.6]';
% params.otherCar.u = car.u_fwd_ref();
% params.otherCar.ref = car.ref_robust();
% result = simulate(params);
% visualization(car, result);

% % 系统参数和初始化
% Ts = 0.1;
% sys = ss([1, Ts; 0, 1], [0; Ts], [], []);
% 
% params = tube_mpc_sets.sys_params(sys, Ts);
% tube_sets = tube_mpc_sets();
% [C_infinity, Vf] = tube_sets.terminal_sets(params);
% 
% % 创建控制器
% mpc_controller = MpcControl_lon();
% mpc_controller.params = params;
% 
% % 设置目标参考值
% ref_velocity = 20; % 目标速度
% disturbance = 0.1; % 扰动估计
% 
% % 初始状态
% x0 = [0; 0];
% 
% % 仿真设置
% sim_time = 10; % 总仿真时间
% Ts = 0.1; % 采样时间
% steps = sim_time / Ts;
% 
% % 仿真
% state = x0;
% trajectory = [];
% for t = 1:steps
%     % 计算控制输入
%     u = mpc_controller.ctrl_opti({state, ref_velocity, 0, disturbance});
% 
%     % 系统更新
%     state = sys.A * state + sys.B * u;
%     trajectory = [trajectory, state];
% end
% 
% % 绘制仿真结果
% time = 0:Ts:sim_time-Ts;
% plot(time, trajectory);
% xlabel('Time (s)');
% ylabel('State');
% legend('Position', 'Velocity');
% title('Tube-MPC Simulation');
% 
