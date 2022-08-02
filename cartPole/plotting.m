close all 
plot_id = 'mc' ; 


pts = 0.25:0.25:2.5 ; 
% pts = 0:20:180 ; 
[r, c] = size(x_trajs) ; 
figure(1) 
xlabel('Time (s)') ; 
ylabel('Angle (rad)') ;
hold on 
legend(num2str(pts)); 
thetas = [] ; 
for i = 1:r 
    data = x_trajs{i,1} ; 
    [R,C] = size(data) ; 
    x = 1:C ; 
    plot(x*0.01, data(3,:)) ; 
        hold on 


    theta = data(3, end) ; 
    thetas = [thetas ; theta] ; 

end
legend('0.25', '0.5', '0.75', '1', '1.25', '1.5', '1.75', '2', '2.25', '2.5'); 
% legend('0', '20', '40', '60', '80', '100', '120', '140', '160', '180'); 
hold off 


figure(2) 
times = x_trajs(:,2) ; 
times = cell2mat(times); 
times(end) = 10e10; 

plot(pts, times,'ko') ; 
xlabel('Uncertainty Factor') ; 
ylabel('Convergance Time (sec)') ; 

ylim([0, 10]) ;
