%% Outer Loop: Attitude and Altitude
figure('name','Tracking Results: Outer Loop');
subplot(2,2,1)
plot(time_plot,roll_ref(index_plot)*rad2deg,'r-','LineWidth',3); hold on;
plot(time_plot,roll(index_plot)*rad2deg,'b-.','LineWidth',2); grid on;
legend('\phi_d','\phi');
title('roll');
xlabel('Time [s]'); ylabel('deg');
subplot(2,2,2)
plot(time_plot,pitch_ref(index_plot)*rad2deg,'r-','LineWidth',3); hold on;
plot(time_plot,pitch(index_plot)*rad2deg,'b-.','LineWidth',2); grid on;
legend('\theta_d','\theta');
title('pitch');
xlabel('Time [s]'); ylabel('deg');
subplot(2,2,3)
plot(time_plot,yaw_ref(index_plot)*rad2deg,'r-','LineWidth',3); hold on;
plot(time_plot,yaw(index_plot)*rad2deg,'b-.','LineWidth',2); grid on;
legend('\psi_d','\psi');
title('yaw');
xlabel('Time [s]'); ylabel('deg'); 
subplot(2,2,4)
plot(time_plot,xi_ref(3,index_plot),'r-','LineWidth',3); hold on;
plot(time_plot,xi(3,index_plot),'b-.','LineWidth',2); grid on;
legend('\xi_{d,z}','\xi_z');
title('altitude');
xlabel('Time [s]'); ylabel('m');