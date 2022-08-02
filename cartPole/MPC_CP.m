%% MPC for Cart-Pole System
function [x_traj, t_converge] = MPC_CP()

global mc ;
global mp ;
global g ;
global l ;
global finished ; 
global timeout ;
% global mc_err ; 


% global I ;
% global b ;
mc = 1 ; % kg 
mp = 0.01 ;% kg 
l = 0.25 ;% m 
g = 9.81 ;% m/s^3 

% Initial Configuration:
xo = zeros(4,1);
xo(1,1) = 0;
xo(2,1) = 0;
xo(3,1) = 0;
xo(4,1) = 0;

% Target: 
p_target(1,1) = 0;
p_target(2,1) = 0;
p_target(3,1) = pi;
p_target(4,1) = 0;

% Weight in Final State:
Q_f = zeros(4,4);
Q_f(1,1) = 0; % x
Q_f(2,2) = 800; % xdot
Q_f(3,3) = 800; % theta
Q_f(4,4) = 800; % thetadot

% Weight in the Control:
R = 20 * eye(1);

% Horizon 
% Horizon = 10; % 1.5sec
% Number of Iterations
% num_iter = 600; 
% Discretization
% dt = 0.02; 

Horizon = 50; % 1.5sec
% Number of Iterations
num_iter = 25; 
% Discretization
dt = 0.01; 

time = [0] ; 
% Learning Rate:
gamma = .8;
% How close current state has to be to target state: 
delta = 0.05;

u = zeros(1, Horizon-2) ; 
u_opt = [];
Cost = [];
x_traj = [];

% horizon = Horizon;
x_curr = xo;
finished = false;

% figure()  
% % title(num2str(mc_err)) ; 
% hold on 
% figure(2)
% hold on
% figure(3) 
% hold on 

% figure()
% hold on 
% axis(gca, 'equal') ; 
% axis([-2.4 2.4 -0.6 0.6]);
% Crank = []; 
% Cart = []; 
% count = 0; 

while ~finished 
    [u, cost] = DDP_CP(x_curr, [u,0] , p_target,Q_f, R, Horizon, num_iter, dt, gamma);
    u_opt = [u_opt u(1,1)];
    Cost = [Cost cost]; 
    x_next = fnsimulate_mpc(x_curr,u(1,1),dt);
    x_traj = [x_traj x_next];
    x_curr = x_next;
    difference =  norm(x_curr(2:end) - p_target(2:end));
%     fprintf("x = %d, v = %d, theta = %d, omega = %d, diff = %d \n", x_curr(1), x_curr(2), x_curr(3)*180/pi, x_curr(4)*180/pi, difference);
    finished = difference < delta;
    
    time(end+1) = time(end) + dt ; 
    if time(end) >= timeout
        break
    end
%     figure(1)  ; 
%     plot(time(end), x_curr(3)*180/pi, 'ro') ; 
%     figure(2)
%     plot(time(end), x_curr(2), 'ko') ; 
%     figure(3)
%     plot(time(end), cost, 'go') ; 
%     drawnow()
% %     plot(time(end),

%     P1 = [x_curr(1),0] ; 
%     theta = x_curr(3) ; 
%     P2 = [P1(1) 0] + l*[sin(theta) -cos(theta)]; 
%     crank = line([P1(1) P2(1)], [P1(2) P2(2)]); 
%     cart = rectangle('Position', [(x_curr(1)-0.2),-0.1, 0.4, 0.2]); 
%     Crank = [Crank ;crank]; 
%     Cart = [Cart ; cart ]; 
%     drawnow() ;  
%     t = time(end) 
%     count = count + 1 ; 
%     if count == 5 
%         
%         for i = 1:5
%             delete(Cart(i)) 
%             delete(Crank(i))
%         end 
%         Cart = [] ; 
%         Crank = [] ; 
%         count = 0; 
% %         axis(gca, 'equal') ; 
%         axis([-500*abs(x_curr(1)) 500*abs(x_curr(1)) -0.6 0.6]);
% %         axis(gca, 'equal') ; 
%     end
    
end
t_converge = NaN;
if finished
   t_converge = time(end); 
end


% hold off 
% hold off
% plot(1:length(x_traj),x_traj)
% legend("x","v","theta","omega")
end
