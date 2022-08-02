function [x] = fnsimulate_mpc(xo,u,dt)

global m ;
global g ;
global l ;
global I ;
global b ; 

x = xo + [xo(2); (-m*g*l*sin(xo(1))/I - b*xo(2)/I + u(1)/I)]*dt ; 

end