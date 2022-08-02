function [x] = fnsimulate_mpc(xo,u,dt)

global mc ;
global mp ;
global g ;
global l ;

global mc_err ; 
global mp_err ; 
global g_err ; 
global l_err ; 

C = mc*mc_err ; 
P = mp*mp_err ; 
G = g*g_err ; 
L = l*l_err ;


theta = xo(3) ; 
theta_dot = xo(4) ; 

% a = ( u(1) + mp*sin(theta)*(l*theta_dot^2 + g*cos(theta)))/(mc + mp*sin(theta)^2) ; 
% q = ( -u(1)*cos(theta) - mp*l*theta_dot^2*cos(theta)*sin(theta) - (mc + mp)*g*sin(theta))/(l*(mc + mp*sin(theta)^2)) ;

a = ( u(1) + P*sin(theta)*(L*theta_dot^2 + G*cos(theta)))/(C + P*sin(theta)^2) ; 
q = ( -u(1)*cos(theta) - P*L*theta_dot^2*cos(theta)*sin(theta) - (C + P)*G*sin(theta))/(L*(C + P*sin(theta)^2)) ;


% a = (u(1)+0.01*sin(xo(3))*(0.25*xo(4)^2+9.81*cos(xo(3))))/(1+0.01*sin(xo(3))^2);
% q = (-u(1)*cos(xo(3))-0.01*0.25*xo(4)^2*cos(xo(3))*sin(xo(3))-(0.01+1)*9.81*sin(xo(3)))/(0.25*(1+0.01*sin(xo(3))^2));

dxdt = zeros(4,1);
dxdt(1,1) = xo(2);
dxdt(2,1) = a;
dxdt(3,1) = xo(4);
dxdt(4,1) = q;

x = xo + dxdt*dt ; 

end