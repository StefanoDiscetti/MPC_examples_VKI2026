% Function to Plot Trajectories and Control Inputs
function plotTrajectories(i, x_MPC, x_LQR, u_MPC, u_LQR,Ts)
    subplot(2, 1, 1);
    plot(x_MPC(1, i), x_MPC(2, i), '.b','Markersize',10);
    hold on
    plot(x_LQR(1, i), x_LQR(2, i), 'or','Markersize',6);
    set(gca,'TickLabelInterpreter','latex','fontsize',10)
    xlim([-0.3 1.5])
    ylim([-0.3 1])
    subplot(2, 1, 2);
    plot((i-1)*Ts, u_MPC(:, i), '.b','Markersize',10);
    hold on
    plot((i-1)*Ts, u_LQR(:, i), 'or','Markersize',6);
    title('Control Inputs','interpreter','latex','fontsize',14);
    xlabel('Time','interpreter','latex','fontsize',12);
    ylabel('Control Input','interpreter','latex','fontsize',12);
    grid on;
    ylim([-2 2])
    set(gca,'TickLabelInterpreter','latex','fontsize',10)
    pause(0.1);
end