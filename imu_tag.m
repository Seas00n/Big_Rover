clear all;clc;close all;
V = diag([0.02, 0.5*pi/180].^2);
veh = Bicycle('covar',V);
% 速度1m/s,转向0.3rad,步长0.1s仿真
% odo为里程计输入
odo = veh.step(1,0.3);
real_x = veh.x';
% 由于里程计输入的噪声，估计值和真实值有一定偏差
estimated_x = veh.f([0,0,0],odo);
veh.add_driver(RandomPath(10))
veh_run = veh.run(1000);
veh.Fx([0,0,0],[0.5,0.1]);
P0 = diag([0.005, 0.005, 0.001].^2);
ekf = EKF (veh, V, P0);
ekf.run(1000);
veh.plot_xy()
hold on
ekf.plot_xy('r')
map = LandmarkMap(20,10);
map.plot()
W = diag([0.1, 1*pi/180].^2);
sensor = RangeBearingSensor(veh, map, 'covar', W);
[z,i] = sensor.reading();
landmark(17)

