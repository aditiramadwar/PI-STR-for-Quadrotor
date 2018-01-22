figure('name','Estimation Results for A');
% estimator for roll dynamics
subplot(2,2,1);
plot(time_plot,a1(1)*ones(1,N+1),'r-.','LineWidth',4); hold on;
plot(time_plot,a2(1)*ones(1,N+1),'g-.','LineWidth',4); hold on;
plot(time_plot,hattheta1(1,index_plot),'b-','LineWidth',2); hold on;
plot(time_plot,hattheta1(2,index_plot),'k-','LineWidth',2); hold on;
title('$A_{\Omega_1}(q)=q^2+a_1q+a_2$','Interpreter','latex','FontSize',14,'FontWeight','normal');
legend({'$a_1$','$a_2$','$\hat{a}_1$','$\hat{a}_2$'},'Interpreter','latex','FontSize',14,'FontWeight','normal');
% estimator for pitch dynamics
subplot(2,2,2);
plot(time_plot,a1(2)*ones(1,N+1),'r-.','LineWidth',4); hold on;
plot(time_plot,a2(2)*ones(1,N+1),'g-.','LineWidth',4); hold on;
plot(time_plot,hattheta2(1,index_plot),'b-','LineWidth',2); hold on;
plot(time_plot,hattheta2(2,index_plot),'k-','LineWidth',2); hold on;
title('$A_{\Omega_2}(q)=q^2+a_1q+a_2$','Interpreter','latex','FontSize',14,'FontWeight','normal');
legend({'$a_1$','$a_2$','$\hat{a}_1$','$\hat{a}_2$'},'Interpreter','latex','FontSize',14,'FontWeight','normal');
% estimator for yaw dynamics
subplot(2,2,3);
plot(time_plot,a1(3)*ones(1,N+1),'r-.','LineWidth',4); hold on;
plot(time_plot,a2(3)*ones(1,N+1),'g-.','LineWidth',4); hold on;
plot(time_plot,hattheta3(1,index_plot),'b-','LineWidth',2); hold on;
plot(time_plot,hattheta3(2,index_plot),'k-','LineWidth',2); hold on;
title('$A_{\Omega_3}(q)=q^2+a_1q+a_2$','Interpreter','latex','FontSize',14,'FontWeight','normal');
xlabel('Time [s]','Interpreter','latex','FontSize',14,'FontWeight','normal');
legend({'$a_1$','$a_2$','$\hat{a}_1$','$\hat{a}_2$'},'Interpreter','latex','FontSize',14,'FontWeight','normal');
% estimator for altitude dynamics
subplot(2,2,4);
plot(time_plot,a1(4)*ones(1,N+1),'r-.','LineWidth',4); hold on;
plot(time_plot,a2(4)*ones(1,N+1),'g-.','LineWidth',4); hold on;
plot(time_plot,hattheta4(1,index_plot),'b-','LineWidth',2); hold on;
plot(time_plot,hattheta4(2,index_plot),'k-','LineWidth',2); hold on;
title('$A_{v_z}(q)=q^2+a_1q+a_2$','Interpreter','latex','FontSize',14,'FontWeight','normal');
xlabel('Time [s]','Interpreter','latex','FontSize',14,'FontWeight','normal');
legend({'$a_1$','$a_2$','$\hat{a}_1$','$\hat{a}_2$'},'Interpreter','latex','FontSize',14,'FontWeight','normal');