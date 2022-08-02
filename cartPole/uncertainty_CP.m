clear all;
close all;
clc

global mc_err ; 
global mp_err ; 
global g_err ; 
global l_err ; 
global finished ;
global timeout ;

mc_err = 1 ; 
mp_err = 0 ;
g_err = 1 ;
l_err = 1 ;
timeout = 10;

finished = false ;
x_trajs = {};

x_v_err = [];
% while 1 
%     close('all')
%     [x_traj, t_converge] = MPC_CP(); 
%     x_trajs = [x_trajs; {x_traj} {t_converge}];
%     if finished 
%         fprintf("MPC converged for mc_err = %d in %d seconds\n", mc_err, t_converge);
%     else
%         fprintf("MPC fails at mc_err = %d \n", mc_err);
%         break;
%     end 
%     
%     mc_err = mc_err + 0.1 ;  
% end

while 1 
    close('all')
    [x_traj, t_converge] = MPC_CP(); 
    x_trajs = [x_trajs; {x_traj} {t_converge}];
    if finished 
        fprintf("MPC converged for mp_err = %d in %d seconds\n", mp_err, t_converge);
    else
        fprintf("MPC fails at mp_err = %d \n", mp_err);
        break;
    end 
    
    mp_err = mp_err + 20 ;  
end