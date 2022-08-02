function [x] = fnsimulate(xo,u_new,Horizon,dt,sigma)

% global m1;
% global m2;
% global s1;
% global s2;
% global I1;
% global I2;
% global b1;
% global b2;
% global b1_2;
% global b2_1;
% global d1;
% global d2;
% global d3;

% global m ;
% global g ;
% global l ;
% global I ;
% global b ;

global mc ;
global mp ;
global g ;
global l ;

x = xo;
u = u_new ; 
for k = 1:(Horizon-1)
    
%     a = ( u(1) + mp*sin(theta)*(l*theta_dot + g*cos(theta)))/(mc + mp*sin(theta)^2) ; 
%     q = ( -u(1)*cos(theta) - mp*l*theta_dot^2*cos(theta) - (mc + mp)*g*sin(theta))/(l*(mc + mp*sin(theta)^2)) ; 
%     
% a = (u(1)+0.01*sin(xo(3))*(0.25*xo(4)^2+9.81*cos(xo(3))))/(1+0.01*sin(xo(3))^2);
% q = (-u(1)*cos(xo(3))-0.01*0.25*xo(4)^2*cos(xo(3))*sin(xo(3))-(0.01+1)*9.81*sin(xo(3)))/(0.25*(1+0.01*sin(xo(3))^2));

%     a = (u_new(1,k)+0.01*sin(x(3,k))*(0.25*x(4,k)^2+9.81*cos(x(3,k))))/(1+0.01*sin(x(3,k))^2);
%     q = (-u_new(1,k)*cos(x(3,k))-0.01*0.25*x(4,k)^2*cos(x(3,k))*sin(x(3,k))-(0.01+1)*9.81*sin(x(3,k)))/(0.25*(1+0.01*sin(x(3,k))^2));
%   
    theta = x(3,k) ; 
    theta_dot = x(4,k) ; 
    a = ( u(1,k) + mp*sin(theta)*(l*theta_dot^2 + g*cos(theta)))/(mc + mp*sin(theta)^2) ; 
    q = ( -u(1,k)*cos(theta) - mp*l*theta_dot^2*cos(theta)*sin(theta) - (mc + mp)*g*sin(theta))/(l*(mc + mp*sin(theta)^2)) ; 
    
    dxdt = zeros(4,1);
    dxdt(1,1) = x(2,k);
    dxdt(2,1) = a;
    dxdt(3,1) = x(4,k);
    dxdt(4,1) = q;
    
x(:,k+1) = x(:,k) + dxdt*dt ; 
end