function [A,B] = fnState_And_Control_Transition_Matrices(x,u,du,dt)
% 
% global d1;
% global d2;
% global d3;
% global b1;
% global b2;
% global b1_2;
% global b2_1;

% global mc ;
% global mp ;
% global g ;
% global l ;

x1 = x(1,1);
x2 = x(2,1);
x3 = x(3,1);
x4 = x(4,1);


u1 = u(1,1);
% u2 = u(2,1);

term1 = (-200*u1*sin(x3)-0.25*x4^2*sin(x3)^2+25*x4^2);
term2 = -9.81*sin(x3)^4-981*sin(x3)^2+(981-9.81*sin(x3)^2)*cos(x3)^2;
term3 = 40500*u1*sin(x3)+100*u1*sin(3*x3)-100.5*x4^2*cos(2*x3);
term4 = -395333*cos(x3)-990.81*cos(3*x3)+0.5*x4^2;

a = (1/(sin(x3)^2+100)^2)*(cos(x3)*term1+term2);
q = (0.005*x4*sin(x3))/(0.01*sin(x3)^2+1);
c = (1/(sin(x3)^2+100)^2)*(term3+term4);
d = -(0.02*x4*sin(x3)*cos(x3))/(0.01*sin(x3)^2+1);
e = 1/(0.01*sin(x3)^2+1);
f = -(4*cos(x3))/(0.01*sin(x3)^2+1);
% m1 = (mp*(cos(x3)*(g*cos(x3) + l*x4^2) - g*sin(x3)^2)*(mc + mp*sin(x3)^2) - ...
%     mp*sin(2*x3)*(u1 + mp*sin(x3)*(l*x4^2 + g*cos(x3))))/(mc + mp*sin(x3)^2)^2 ; 
% 
% m2 = 2*l*mp*x4*sin(x3)/(mc + mp*sin(x3)^2) ; 
% 
% m3 = ((u1*sin(x3) - l*mp*x4^2*cos(2*x3) - g*cos(x3)*(mc+mp))*(mc + mp*sin(x3)^2) - ... 
%      mp*sin(2*x3)*(-u1*cos(x3) - (l*mp*x4^2 * sin(2*x3))/(2) - g*sin(x3)*(mc+mp))) / ... 
%      (l*(mc + mp*sin(x3)^2))^2 ; 
% m4 = -mp*x4*sin(2*x3)/(mc + mp*sin(x3)^2) ; 

A = [0 1 0 0;...
    0 0 a q;...
    0 0 0 1;...
    0 0 c d];

% A = [0 1 0 0; ... 
%      0 0 m1 m2; ... 
%      0 0 0 1; ... 
%      0 0 m3 m4]; 
% 
% m5 = 1/(mc + mp*sin(x3)^2) ; 
% 
% m6 = -cos(x3)/(l*(mc + mp*sin(x3)^2)) ; 
B = [0;...
    e ;...
    0;...
    f]; 

% B = [0;...
%      m5 ;...
%      0;...
%      m6]; 
end 




