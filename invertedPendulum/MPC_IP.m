%% MPC for Inverted Pendulum System

clear all;
close all;
clc

global m ;
global g ;
global l ;
global I ;
global b ; 

timer = tic;

% Inverted Pendulum Model Parameter 
m = 1 ; % kg 
g = 9.81 ; % m/s^2 
l = 1 ; % m 
I = m*l^2 ; %kgm^2 
b = 0.25 ; 

% Initial Configuration:
xo = zeros(2,1);
xo(1,1) = 0;
xo(2,1) = 0;


% Target: 
p_target(1,1) = pi;
p_target(2,1) = 0;

% Weight in Final State:
Q_f = zeros(2,2);
Q_f(1,1) = 5000;
Q_f(2,2) = 5000;

% Weight in the Control:
R = 5 * eye(1);

% Horizon 
Horizon = 100; % 1.5sec
% Number of Iterations
num_iter = 25;
% Discretization
dt = 0.01;
% Learning Rate:
gamma = 0.1;
% How close current state has to be to target state: 
delta = 0.05;

u = zeros(1,Horizon-2);

u_opt = [];
Cost = [];
x_traj = [];

% horizon = Horizon;
x_curr = xo;
finished = false;
% attempts = 0;
while ~finished
% while ~finished && horizon ~= 1
    [u, cost] = DDP_IP(x_curr, [u 0], p_target, Q_f, R, Horizon, num_iter, dt, gamma);
    u_opt = [u_opt u(1,1)];
    Cost = [Cost cost]; 
    x_next = fnsimulate_mpc(x_curr,u(1,1),dt);
    x_traj = [x_traj x_next];
    x_curr = x_next;
%     x_curr(1) = mod(x_curr(1),2*pi);
    difference =  norm(x_curr-p_target);
    fprintf("theta = %d, omega = %d \n", x_curr(1)*180/pi, x_curr(2)*180/pi);
    finished = difference < delta;
%     horizon = horizon - 1;
%     attempts = attempts+1;
end
subplot(1,2,1)
plot(1:length(x_traj),x_traj,'LineWidth',3)
time_elapsed = toc(timer);
legend("theta","omega")
xlabel

subplot(1,2,2) 
plot(1:length(Cost),Cost) ; 
legend('Cost')
