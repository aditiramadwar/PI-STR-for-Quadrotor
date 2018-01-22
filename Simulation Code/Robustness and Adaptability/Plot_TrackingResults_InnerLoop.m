%% Inner Loop: Angular Velocity and Linear Velocity in z-direction
switch controller_chosen
    case 1
        figure('name','Tracking Results: Inner Loop')
        subplot(2,2,1)
        plot(time_plot, dot_y_ref(1,index_plot),'r-','LineWidth',3); hold on;
        plot(time_plot, Omega(1,index_plot)*rad2deg,'b-.','LineWidth',2); grid on;
        legend('\Omega_1^d','\Omega_1');
        title('\Omega_1');
        xlabel('Time [s]'); ylabel('deg/s');
        subplot(2,2,2)
        plot(time_plot, dot_y_ref(2,index_plot),'r-','LineWidth',3); hold on;
        plot(time_plot, Omega(2,index_plot)*rad2deg,'b-.','LineWidth',2); grid on;
        legend('\Omega_2^d','\Omega_2');
        title('\Omega_2');
        xlabel('Time [s]'); ylabel('deg/s');
        subplot(2,2,3)
        plot(time_plot, dot_y_ref(3,index_plot),'r-','LineWidth',3); hold on;
        plot(time_plot, Omega(3,index_plot)*rad2deg,'b-.','LineWidth',2); grid on;
        legend('\Omega_3^d','\Omega_3');
        title('\Omega_3');
        xlabel('Time [s]'); ylabel('deg/s');
        subplot(2,2,4)
        plot(time_plot, dot_y_ref(4,index_plot),'r-','LineWidth',3); hold on;
        plot(time_plot, v(3,index_plot),'b-.','LineWidth',2); grid on;
        legend('v_z^d','v_z');
        title('v_z');
        xlabel('Time [s]'); ylabel('deg/s');
    case 2
        figure('name','Tracking Results: Inner Loop')
        subplot(2,2,1)
        plot(time_plot, u_c_c2(1,index_plot),'r-','LineWidth',3); hold on;
        plot(time_plot, Omega(1,index_plot)*rad2deg,'b-.','LineWidth',2); grid on;
        legend('\Omega_1^d','\Omega_1');
        title('\Omega_1');
        xlabel('Time [s]'); ylabel('deg/s');
        subplot(2,2,2)
        plot(time_plot, u_c_c2(2,index_plot),'r-','LineWidth',3); hold on;
        plot(time_plot, Omega(2,index_plot)*rad2deg,'b-.','LineWidth',2); grid on;
        legend('\Omega_2^d','\Omega_2');
        title('\Omega_2');
        xlabel('Time [s]'); ylabel('deg/s');
        subplot(2,2,3)
        plot(time_plot, u_c_c2(3,index_plot),'r-','LineWidth',3); hold on;
        plot(time_plot, Omega(3,index_plot)*rad2deg,'b-.','LineWidth',2); grid on;
        legend('\Omega_3^d','\Omega_3');
        title('\Omega_3');
        xlabel('Time [s]'); ylabel('deg/s');
        subplot(2,2,4)
        plot(time_plot, u_c_c2(4,index_plot),'r-','LineWidth',3); hold on;
        plot(time_plot, v(3,index_plot),'b-.','LineWidth',2); grid on;
        legend('v_z^d','v_z');
        title('v_z');
        xlabel('Time [s]'); ylabel('deg/s');
end