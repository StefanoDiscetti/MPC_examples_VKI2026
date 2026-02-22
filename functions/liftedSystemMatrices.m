% Function to Construct Lifted System Matrices
function [A_, B_, Q_] = liftedSystemMatrices(A, B, Q, N)
    A_ = zeros(size(A,1)*(N+1),size(A,2)); 
    for i = 0:N
        A_((i*size(A,1)+1):(i+1)*size(A,1),:) = A^i;
    end    

    B_ = zeros(size(A,1)*N, size(B,2)*N);
    for i = 1:N
        for j = 1:i
            B_(size(A,1)*(i-1)+1:size(A,1)*i, size(B,2)*(j-1)+1:size(B,2)*j) = A^(i-j) * B;
        end
    end
    B_ = [zeros(size(B,1), size(B,2)*N); B_];

    Q_ = kron(eye(N+1),Q);
end
