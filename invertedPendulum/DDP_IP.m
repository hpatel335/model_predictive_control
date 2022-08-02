function [u, cost] = DDP_IP(x_curr, u_init, p_target, Q_f, R, Horizon, num_iter, dt, gamma)








% Initial Configuration:
xo = x_curr;

% Initial Control:
u_k = u_init;
du_k = zeros(1,Horizon-1);

% Initial trajectory:
x_traj = zeros(2,Horizon);
% x_traj(:,1) = xo;

for k = 1:num_iter

%------------------------------------------------> Linearization of the dynamics
%------------------------------------------------> Quadratic Approximations of the cost function 
for  j = 1:(Horizon-1)
    
     [l0,l_x,l_xx,l_u,l_uu,l_ux] = fnCost(x_traj(:,j), u_k(:,j), j,R,dt);
      L(j) = dt * l0;
      L_x(:,j) = dt * l_x;
      L_xx(:,:,j) = dt * l_xx;
      L_u(:,j) = dt * l_u;
      L_uu(:,:,j) = dt * l_uu;
      L_ux(:,:,j) = dt * l_ux; 
    
    [dfx,dfu] = fnState_And_Control_Transition_Matrices(x_traj(:,j),u_k(:,j),du_k(:,j),dt);
   
    phi(:,:,j) = eye(2,2) + dfx * dt; %phi 
    B(:,:,j) = dfu * dt;  
end

%------------------------------------------------> Find the controls
Vxx(:,:,Horizon)= Q_f;
Vx(:,Horizon) = Q_f * (x_traj(:,Horizon) - p_target); 
V(Horizon) = 0.5 * (x_traj(:,Horizon) - p_target)' * Q_f * (x_traj(:,Horizon) - p_target); 

%------------------------------------------------> Backpropagation of the Value Function
for j = (Horizon-1):-1:1
     Q_o = L(j) + V(j+1) ; 
     Q_u = L_u(:,j) + B(:,:,j)'*Vx(:, j+1) ; 
     Q_x = L_x(:,j) + phi(:,:,j)'*Vx(:, j+1) ; 
     Q_xx = L_xx(:,:,j) + phi(:,:,j)'*Vxx(:,:,j+1)*phi(:,:,j) ; 
     Q_uu = L_uu(:,:,j) + B(:,:,j)'*Vxx(:,:,j+1)*B(:,:,j) ; 
     Q_ux = L_ux(:,:,j) + B(:,:,j)'*Vxx(:,:,j+1)*phi(:,:,j) ; 
     Q_xu = Q_ux' ; 
     
     l_k(:,j) = -inv(Q_uu)*Q_u ; 
     L_k(:,:,j) = -inv(Q_uu)*Q_ux ; 
     
     Vxx(:, :, j) = Q_xx - Q_xu*inv(Q_uu)*Q_ux ; 
     Vx(:,j) = Q_x - Q_xu*inv(Q_uu)*Q_u ; 
     V(:, j) = Q_o - 0.5*Q_u'*inv(Q_uu)*Q_u ; 
     
end 


%----------------------------------------------> Find the controls
dx = zeros(2,1);

for i=1:(Horizon-1)    
   du = l_k(:,i) + L_k(:,:,i) * dx;
   dx = phi(:,:,i) * dx + B(:,:,i) * du;  
   u_new(:,i) = u_k(:,i) + gamma * du;
end

u_k = u_new;


%---------------------------------------------> Simulation of the Nonlinear System
[x_traj] = fnsimulate(xo,u_new,Horizon,dt,0);
[Cost(:,k)] =  fnCostComputation(x_traj,u_k,p_target,dt,Q_f,R);
% x1(k,:) = x_traj(1,:);
 

% fprintf('Iteration %d,  Current Cost = %e \n',k,Cost(1,k));
end

%% New stuff
u = u_new;
cost = Cost(:,end);