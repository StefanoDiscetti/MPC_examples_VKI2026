%-------------------------------------------------------------------%
% Example 1 - Model Predictive Control vs LQR for a Linear System
% Author: S. Discetti, Jan 2024
%-------------------------------------------------------------------%

clc; clear; close all;
addpath('functions')

% Definition of Continuous LDTI System
A = [1 -1; 1 1];   
B = [0.1; 1];
C = eye(2);
D = zeros(size(B));
sys = ss(A, B, C, D);

% Conversion to Discrete System
Ts = 0.3;                           % Sampling time
sysd = c2d(sys, Ts);                % Conversion to discrete system
[A, B] = deal(sysd.A, sysd.B);

% MPC Parameters
N = 2;                             % Prediction horizon (timesteps)
Q = eye(size(A));                   % State cost matrix
R = 1;                              % Control cost
steps = 50;                         % Number of simulation steps

% Initial State
x0 =[sqrt(2)/2; sqrt(2)/2];    
n = numel(x0);                      % state vector size
m = size(B,2);                      % input vector size

% LQR Optimal Feedback Control
[K, S, e] = dlqr(A, B, Q, R);       
[P,~,~] = idare(A, B, Q, R, [], []);

% Initialize States
x = x0;
xLQR = x0;
x_LQR=zeros(n,steps);
u_LQR=zeros(1,steps);
x_MPC=x_LQR;
u_MPC=u_LQR;
% Plot Initial Points
figure;
subplot(2,1,1);
plot(x(1), x(2), 'pb', 'Markersize', 10);
hold on;
plot(0, 0, 'kx', 'Markersize', 10);
title('State Trajectories','interpreter','latex','fontsize',14);
xlabel('State 1','interpreter','latex','fontsize',12);
ylabel('State 2','interpreter','latex','fontsize',12);
grid on;


% Simulation Loop
for i = 1:steps
    % LQR Control
    uLQR = -K * xLQR;
    xLQR = A * xLQR + B * uLQR;
    x_LQR(:, i) = xLQR;
    u_LQR(:, i) = uLQR;

    % MPC Control
    [U, xMPC] = solveMPC_terminal(A, B, Q, R, N, x,P);
    x = xMPC(:, 2);
    x_MPC(:,i)=x;
    u_MPC(:,i) = U(1);
    
    % Plotting
    plotTrajectories(i, x_MPC, x_LQR, u_MPC, u_LQR,Ts);
end




subplot(2,1,1)
plot([x0(1) x_MPC(1,:)],[x0(2) x_MPC(2,:)], '-b','linewidth',1.5);
hold on
plot([x0(1) x_LQR(1,:)],[x0(2) x_LQR(2,:)], '-r','linewidth',1.5);
h=legend('Initial State','Origin','MPC', 'LQR', 'Location', 'best');
set(h,'interpreter','latex','fontsize',12)
legend boxoff

subplot(2,1,2)
h=legend('MPC', 'LQR', 'Location', 'NorthEast');
set(h,'interpreter','latex','fontsize',12)
legend boxoff
legendLabels = get(h, 'String');



figure
subplot(3,1,1)
plot((0:steps)*Ts,[x0(1,1) x_MPC(1,:)],'.b','Markersize',10);
hold on
plot((0:steps)*Ts,[x0(1,1) x_LQR(1,:)],'or','Markersize',6);
title('State 1','interpreter','latex','fontsize',14);
xlabel('Time','interpreter','latex','fontsize',12);
ylabel('$x_1$','interpreter','latex','fontsize',12);
grid on;
ylim([-2 2])
set(gca,'TickLabelInterpreter','latex','fontsize',10)

subplot(3,1,2)
plot((0:steps)*Ts,[x0(2,1) x_MPC(2,:)],'.b','Markersize',10);
hold on
plot((0:steps)*Ts,[x0(2,1) x_LQR(2,:)],'or','Markersize',6);
title('State 2','interpreter','latex','fontsize',14);
xlabel('Time','interpreter','latex','fontsize',12);
ylabel('$x_2$','interpreter','latex','fontsize',12);
grid on;
ylim([-2 2])
set(gca,'TickLabelInterpreter','latex','fontsize',10)


subplot(3,1,3)
plot((0:steps-1)*Ts,u_MPC,'.b','Markersize',10);
hold on
plot((0:steps-1)*Ts,u_LQR,'or','Markersize',6);
title('Control Inputs','interpreter','latex','fontsize',14);
xlabel('Time','interpreter','latex','fontsize',12);
ylabel('Control Input','interpreter','latex','fontsize',12);
grid on;
ylim([-2 2])
set(gca,'TickLabelInterpreter','latex','fontsize',10)



figure
subplot(3,1,1)
plot((0:steps)*Ts,[x0(1,1) x_MPC(1,:)],'-b','Linewidth',1.5);
hold on
plot((0:steps)*Ts,[x0(1,1) x_LQR(1,:)],'--r','Linewidth',1.5);
title('State 1','interpreter','latex','fontsize',14);
xlabel('Time','interpreter','latex','fontsize',12);
ylabel('$x_1$','interpreter','latex','fontsize',12);
grid on;
ylim([-0.5 1])
set(gca,'TickLabelInterpreter','latex','fontsize',10)
h=legend('MPC', 'LQR', 'Location', 'NorthEast');
set(h,'interpreter','latex','fontsize',12)
legend boxoff

subplot(3,1,2)
plot((0:steps)*Ts,[x0(2,1) x_MPC(2,:)],'-b','Linewidth',1.5);
hold on
plot((0:steps)*Ts,[x0(2,1) x_LQR(2,:)],'--r','Linewidth',1.5);
title('State 2','interpreter','latex','fontsize',14);
xlabel('Time','interpreter','latex','fontsize',12);
ylabel('$x_2$','interpreter','latex','fontsize',12);
grid on;
ylim([-0.5 1])
set(gca,'TickLabelInterpreter','latex','fontsize',10)
h=legend('MPC', 'LQR', 'Location', 'NorthEast');
set(h,'interpreter','latex','fontsize',12)
legend boxoff


subplot(3,1,3)
plot((0:steps-1)*Ts,u_MPC,'-b','Linewidth',1.5);
hold on
plot((0:steps-1)*Ts,u_LQR,'--r','Linewidth',1.5);
title('Control Inputs','interpreter','latex','fontsize',14);
xlabel('Time','interpreter','latex','fontsize',12);
ylabel('Control Input','interpreter','latex','fontsize',12);
grid on;
ylim([-2 2])
set(gca,'TickLabelInterpreter','latex','fontsize',10)
h=legend('MPC', 'LQR', 'Location', 'NorthEast');
set(h,'interpreter','latex','fontsize',12)
legend boxoff