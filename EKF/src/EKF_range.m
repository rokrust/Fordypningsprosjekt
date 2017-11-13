%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%          Code written by Pål Mathisen
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [P_pri_next, x_hat_next, x_hat_post, error] = EKF_range(x_hat, sig, y, u, P_pri, A, B, R, Q, H_EK)
    %[P_pri_next, x_hat_next, x_hat_post] = EKF_range(x_hat, sig, y, u, P_pri, A, B, R, Q, H_EK)
    
    %%Prework
    n = size(x_hat,1);
    y_hat = h_EK(x_hat, sig);
    n_y = size(y_hat,1);
    n_u = size(u,1);
    H = H_EK(x_hat(1:3), sig, y_hat); %G
    
    %%Simple validity checks
    %{
    if rank(obsv(A,H)) ~= 9
        error('Not observable');
    end
    
    if (size(A,1) ~= n || size(A,2) ~= n)
        error('A is not right dimensions.');
    elseif (size(B,1) ~= n || size(B,2) ~= n_u)
        error('B is not right dimensions.');
    elseif (size(P_pri,1) ~= n || size(P_pri,2) ~= n)
        error('P_pri is not right dimensions.');
    elseif (size(Q,1) ~= n || size(Q,2) ~= n)
        error('Q is not right dimensions.');
    elseif (size(R,1) ~= n_y || size(R,2) ~= n_y)
        error('R is not right dimensions.');
    elseif (size(H,1) ~= n_y || size(H,2) ~= n)
        error ('H is not right dimensions.');
    end
    %}
    
    
    %%Corrector
    K                       = P_pri*H'/(H*P_pri*H' + R);
    error                   = y-y_hat;
    x_hat_post              = x_hat + K*(error);
    P_post                  = (eye(n) - K*H)*P_pri*(eye(n) - K*H)' + K*R*K';
    P_post                  = (P_post + P_post')/2;
    
    %%Predictor
    x_hat_next              = A*x_hat_post + B*u;
    P_pri_next              = A*P_post*A' + Q;
end