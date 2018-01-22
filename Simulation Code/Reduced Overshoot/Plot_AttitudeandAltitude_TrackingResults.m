%% plottrackingresults
figure('name','Tracking Results');
subplot(2,1,1)
plot(time_plot, zeros(1,length(index_plot)),'r-','LineWidth',4); hold on;
plot(time_plot,roll(index_plot)*rad2deg,'g-.','LineWidth',2); hold on;
plot(time_plot,pitch(index_plot)*rad2deg,'k--','LineWidth',2); hold on;
plot(time_plot,yaw(index_plot)*rad2deg,'b-','LineWidth',3); grid on;
title('Attitude--Euler Angles','Fontname', 'Times New Roman','FontSize',10);
ylabel('degree','Fontname', 'Times New Roman','FontSize',10);
legend({'zero','\phi','\theta','\psi'},'Fontname', 'Times New Roman','FontSize',10);
% altitude
subplot(2,1,2)
plot(time_plot,altitude_ref(index_plot),'r-.','LineWidth',4); hold on;
plot(time_plot,altitude(index_plot),'b-','LineWidth',3); grid on;
title('Altitude','Fontname', 'Times New Roman','FontSize',10); 
xlabel('Time [s]','Fontname', 'Times New Roman','FontSize',10);
ylabel('m','Fontname', 'Times New Roman','FontSize',10);
legend({'-\xi_{d,z}','-\xi_{z}'},'Fontname', 'Times New Roman','FontSize',10);