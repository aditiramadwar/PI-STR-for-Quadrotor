%% Load Data
load data_PID.mat;
load data_PI_STR.mat;
%% Compare: Trajectory
figure('name','Compare: Trajectory')
h1 = plot3(xi_ref(1,index_plot),xi_ref(2,index_plot),-xi_ref(3,index_plot),'r-','LineWidth',4); hold on;
h2 = plot3(xi_PID(1,index_plot),xi_PID(2,index_plot),-xi_PID(3,index_plot),'b-','LineWidth',2); hold on;
h3 = plot3(xi_PI_STR(1,index_plot),xi_PI_STR(2,index_plot),-xi_PI_STR(3,index_plot),'k-','LineWidth',2); hold on;
% marker
num_marker1 = 12;
for i=1:num_marker1
    h4 = plot3(xi_PID(1,i*N/num_marker1),xi_PID(2,i*N/num_marker1),-xi_PID(3,i*N/num_marker1),'b-o','LineWidth',2); hold on;
end
num_marker2 = 8;
for i=1:num_marker2
    h5 = plot3(xi_PI_STR(1,i*N/num_marker2),xi_PI_STR(2,i*N/num_marker2),-xi_PI_STR(3,i*N/num_marker2),'k-p','LineWidth',2); hold on;
end
legend([h1,h4,h5],{'Desired','PID','PI-STR'},'Fontname', 'Times New Roman','FontSize',10);
grid on;
title('Trajectory','Fontname', 'Times New Roman','FontSize',10);
xlabel('\xi_{x} (m)','Fontname', 'Times New Roman','FontSize',10); 
ylabel('\xi_{y} (m)','Fontname', 'Times New Roman','FontSize',10); 
zlabel('-\xi_{z} (m)','Fontname', 'Times New Roman','FontSize',10);
%% Compare: Altitude
figure('name','Compare: Altitude')
h1 = plot(time_plot,-xi_ref(3,index_plot),'r-.','LineWidth',4); hold on;
h2 = plot(time_plot,-xi_PID(3,index_plot),'b-.','LineWidth',2); hold on;
h3 = plot(time_plot,-xi_PI_STR(3,index_plot),'k-','LineWidth',2); hold on;
num_marker2 = 8;
for i=1:num_marker2
    h5 = plot(time_plot(i*N/num_marker2),-xi_PI_STR(3,i*N/num_marker2),'k-p','LineWidth',2); hold on;
end
legend([h1,h2,h5],{'Desired','PID','PI-STR'},'Fontname', 'Times New Roman','FontSize',10);
grid on;
title('Altitude: -\xi_z','Fontname', 'Times New Roman','FontSize',10);
xlabel('Time [s]','Fontname', 'Times New Roman','FontSize',10);
ylabel('m','Fontname', 'Times New Roman','FontSize',10);
if state_add_dist_wind == 1
    annotation('textarrow',[93.3/240,93.3/240],[15/40,25/40],'String','add wind gusts','Fontname', 'Times New Roman','FontSize',10);
    annotation('textarrow',[155.3/240,155.3/240],[15/40,25/40],'String','remove wind gusts','Fontname', 'Times New Roman','FontSize',10);
end
if state_change_m == 1
    annotation('textarrow',[93.3/240,93.3/240],[15/40,25/40],'String','add 300-g load','Fontname', 'Times New Roman','FontSize',10);
    annotation('textarrow',[116.3/240,116.3/240],[34/40,25/40],'String','remove 300-g load','Fontname', 'Times New Roman','FontSize',10);
    annotation('textarrow',[140.3/240,140.3/240],[15/40,25/40],'String','remove 300-g load','Fontname', 'Times New Roman','FontSize',10);
    annotation('textarrow',[175.3/240,163.3/240],[34/40,25/40],'String','add 300-g load','Fontname', 'Times New Roman','FontSize',10);
end
%% Compare: Position
figure('name','Compare: Position')
subplot(3,1,1)
plot(time_plot,yaw_ref(index_plot)*rad2deg,'r-.','LineWidth',4); hold on;
plot(time_plot,yaw_PID(index_plot)*rad2deg,'b-','LineWidth',2); hold on;
plot(time_plot,yaw_PI_STR(index_plot)*rad2deg,'k-','LineWidth',2); grid on;
legend({'Desired','PID','PI-STR'},'Fontname', 'Times New Roman','FontSize',10);
title('Yaw Angle: \psi','Fontname', 'Times New Roman','FontSize',10);
ylabel('degree','Fontname', 'Times New Roman','FontSize',10);
subplot(3,1,2)
plot(time_plot,xi_ref(1,index_plot),'r-.','LineWidth',4); hold on;
plot(time_plot,xi_PID(1,index_plot),'b-','LineWidth',2); hold on;
plot(time_plot,xi_PI_STR(1,index_plot),'k-','LineWidth',2); grid on;
legend({'Desired','PID','PI-STR'},'Fontname', 'Times New Roman','FontSize',10);
title('\xi_x');
ylabel('m');
subplot(3,1,3)
plot(time_plot,xi_ref(2,index_plot),'r-.','LineWidth',4); hold on;
plot(time_plot,xi_PID(2,index_plot),'b-','LineWidth',2); hold on;
plot(time_plot,xi_PI_STR(2,index_plot),'k-','LineWidth',2); grid on;
legend({'Desired','PID','PI-STR'},'Fontname', 'Times New Roman','FontSize',10);
title('\xi_y','Fontname', 'Times New Roman','FontSize',10);
xlabel('Time [s]','Fontname', 'Times New Roman','FontSize',10);
ylabel('m','Fontname', 'Times New Roman','FontSize',10);
%% Compare: Attitude
figure('name','Compare: Attitude')
subplot(2,1,1)
plot(time_plot,roll_ref(index_plot)*rad2deg,'r-.','LineWidth',4); hold on;
plot(time_plot,roll_PID(index_plot)*rad2deg,'b-','LineWidth',2); hold on;
plot(time_plot,roll_PI_STR(index_plot)*rad2deg,'k-','LineWidth',2); grid on;
legend({'Desired','PID','PI-STR'},'Fontname', 'Times New Roman','FontSize',10);
title('Roll Angle: \phi','Fontname', 'Times New Roman','FontSize',10);
xlabel('Time [s]','Fontname', 'Times New Roman','FontSize',10); 
ylabel('degree','Fontname', 'Times New Roman','FontSize',10);
subplot(2,1,2)
plot(time_plot,pitch_ref(index_plot)*rad2deg,'r-.','LineWidth',4); hold on;
plot(time_plot,pitch_PID(index_plot)*rad2deg,'b-','LineWidth',2); hold on;
plot(time_plot,pitch_PI_STR(index_plot)*rad2deg,'k-','LineWidth',2); grid on;
legend({'Desired','PID','PI-STR'},'Fontname', 'Times New Roman','FontSize',10);
title('Pitch Angle: \theta','Fontname', 'Times New Roman','FontSize',10);
xlabel('Time [s]','Fontname', 'Times New Roman','FontSize',10); 
ylabel('degree','Fontname', 'Times New Roman','FontSize',10);
%% Lateral with Roll
figure('name','Compare: Lateral with Roll')
h1 = plot(time_plot,xi_ref(2,index_plot),'r-.','LineWidth',4); hold on;
h2 = plot(time_plot,xi_PID(2,index_plot),'b-.','LineWidth',2); hold on;
h3 = plot(time_plot,xi_PI_STR(2,index_plot),'k-','LineWidth',2); hold on;
num_marker2 = 8;
for i=1:num_marker2
    h5 = plot(time_plot(i*N/num_marker2),xi_PI_STR(2,i*N/num_marker2),'k-p','LineWidth',2); hold on;
end
legend([h1,h2,h5],{'Desired','PID','PI-STR'},'Fontname', 'Times New Roman','FontSize',10);
grid on;
title('Lateral: \xi_y','Fontname', 'Times New Roman','FontSize',10);
xlabel('Time [s]','Fontname', 'Times New Roman','FontSize',10);
ylabel('m','Fontname', 'Times New Roman','FontSize',10);
%% Compare Inputs
FontSize = 18;
figure('name','Input Voltages')
subplot(2,1,1)
plot(time_plot,Umax_motor(index_plot),'r-','linewidth',4); hold on;
for i=1:4
    plot(time_plot,U_motor_PID(i,index_plot),'linewidth',2); hold on;
end
grid on;
title('Input Voltages: PID', ...
    'Interpreter', 'LaTex', ...
    'Fontname', 'Times', ...
    'FontSize', FontSize);
ylabel('V', ...
    'Fontname', 'Times', ...
    'FontSize', FontSize);
set(gca, ...
    'Fontname', 'Times', ...
    'FontSize', FontSize-2);
subplot(2,1,2)
plot(time_plot,Umax_motor(index_plot),'r-','linewidth',4); hold on;
for i=1:4
    plot(time_plot,U_motor_PI_STR(i,index_plot),'linewidth',2); hold on;
end
grid on;
legend1 = legend({'$U_{\max}$','$U_1$','$U_2$','$U_3$','$U_4$'},...
    'Interpreter', 'LaTex', ...
    'Fontname', 'Times', ...
    'FontSize', 12);
set(legend1,...
    'Position',[0.147738093960853 0.2954818280653 0.152500001464571 0.319047626313709],...
    'FontSize',12);
title('Input Voltages: PI-STR', ...
    'Interpreter', 'LaTex', ...
    'Fontname', 'Times', ...
    'FontSize', FontSize);
xlabel('Time [s]',...
    'Fontname', 'Times', ...
    'FontSize', FontSize);
ylabel('V', ...
    'Fontname', 'Times', ...
    'FontSize', FontSize);
set(gca, ...
    'Fontname', 'Times', ...
    'FontSize', FontSize-2);
