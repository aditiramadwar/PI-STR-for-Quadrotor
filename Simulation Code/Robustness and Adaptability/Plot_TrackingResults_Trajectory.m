%% Trajectory
figure('name','Trajectory')
plot3(xi_ref(1,index_plot),xi_ref(2,index_plot),-xi_ref(3,index_plot),'r:','LineWidth',3); hold on;
plot3(xi(1,index_plot),xi(2,index_plot),-xi(3,index_plot),'b-.','LineWidth',2); hold on;
grid on;
switch controller_chosen
    case 1
        legend('Desired','c1-PID');
    case 2
        legend('Desired','c1-PI-STR');
end
title('Trajectory');
xlabel('m'); ylabel('m'); zlabel('m');
%% Position
figure('name','Position')
subplot(2,2,1)
plot(time_plot,xi_ref(1,index_plot),'r-','LineWidth',3); hold on;
plot(time_plot,xi(1,index_plot),'b-.','LineWidth',2); hold on;
grid on;
legend('\xi_{d,x}','\xi_{x}');
title('\xi_x');
xlabel('Time [s]'); ylabel('m');
subplot(2,2,2)
plot(time_plot,xi_ref(2,index_plot),'r-','LineWidth',3); hold on;
plot(time_plot,xi(2,index_plot),'b-.','LineWidth',2); hold on;
grid on;
legend('\xi_{d,y}','\xi_{y}');
title('\xi_y');
xlabel('Time [s]'); ylabel('m');
subplot(2,2,3)
plot(time_plot,xi_ref(3,index_plot),'r-','LineWidth',3); hold on;
plot(time_plot,xi(3,index_plot),'b-.','LineWidth',2); hold on;
grid on;
legend('\xi_{d,z}','\xi_{z}');
title('\xi_z');
xlabel('Time [s]'); ylabel('m');
subplot(2,2,4)
plot(time_plot, dot_xi_ref(3,index_plot),'r-','LineWidth',3); hold on;
plot(time_plot, v(3,index_plot),'b-.','LineWidth',2); grid on;
legend('v_{d,z}','v_z');
title('v_z');
xlabel('Time [s]'); ylabel('m/s'); 
%% ESO
figure('name','ESO')
subplot(2,1,1)
plot(time_plot,roll_ref(index_plot)*rad2deg,'r','LineWidth',3); hold on;
plot(time_plot,hat_roll_ref(index_plot)*rad2deg,'b-.','LineWidth',2); hold on;
grid on;
title('ESO: \phi');
legend('true','hat');
xlabel('Time [s]'); ylabel('deg');
subplot(2,1,2)
plot(time_plot,pitch_ref(index_plot)*rad2deg,'r','LineWidth',3); hold on;
plot(time_plot,hat_pitch_ref(index_plot)*rad2deg,'b-.','LineWidth',2); hold on;
grid on;
title('ESO: \theta');
legend('true','hat');
xlabel('Time [s]'); ylabel('deg');
%% Motor
figure('name','Motor Speeds')
plot(time_plot,w_motor_N*ones(1,N+1),'r-','LineWidth',3); hold on;
plot(time_plot,w_motor(1,index_plot),'b-','LineWidth',2); hold on;
plot(time_plot,w_motor(2,index_plot),'k-','LineWidth',2); hold on;
plot(time_plot,w_motor(3,index_plot),'b--','LineWidth',1); hold on;
plot(time_plot,w_motor(4,index_plot),'k--','LineWidth',1); hold on;
grid on;
legend({'\varpi_N','\varpi_1','\varpi_2','\varpi_3','\varpi_4'});
title('Motor Speeds'); 
xlabel('Time [s]'); ylabel('rad/s');