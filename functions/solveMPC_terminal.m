% Function to Solve MPC Problem
function [U, x_MPC] = solveMPC_terminal(A, B, Q, R, N, x,P, u_min, u_max,x_min,x_max)
    [A_, B_, Q_] = liftedSystemMatrices_terminal(A, B, Q, N,P);
    
    F=[];
    G=[];
    if nargin>=9
        Fu=[1;-1];
        gu=[u_max;-u_min];
    end
    if nargin==11
        Fx=[1 0;0 1; -1 0; 0 -1];
        gx=[x_max;x_max;-x_min;-x_min];
    end
    

    % Quadratic Program Matrices
    H = B_' * Q_ * B_ + kron(eye(N), R);
    V = A_' * Q_ * B_;
    H = (H + H') / 2;               % Ensuring H is symmetric
    
    % Input Constraints
    if nargin==9
        G = [repmat(gu,N,1)];
        F = [blkdiag(kron(eye(N),Fu))];
    end
    if nargin==11
        G = [repmat(gx,(N+1),1)-blkdiag(kron(eye(N+1),Fx))*A_*x;repmat(gu,N,1)];
        F = [blkdiag(kron(eye(N+1),Fx))*B_; blkdiag(kron(eye(N),Fu))];
    end
    
    % Solving Quadratic Program with Constraints
    U = quadprog(H, V' * x, F, G, [], [], [], []);
    
    % Extracting MPC trajectory
    x_MPC = reshape(A_ * x + B_ * U, size(A,1), N+1);
end
