%{
function [ekf] = EKF_step(sat_poss, y, u, ekf)
    %%Validity checks
    
    %%Initial variables
    n_sat = size(y, 1);
    ekf.R = eye(n_sat);
    
    %%Predict
    %State estimate
    x_hat_ = ekf.A*ekf.x_hat + ekf.B*u;
    
    %Estimated range
    ekf.H = [ekf.h(x_hat_, sat_poss, y), zeros(n_sat, ekf.cfg.n - ekf.cfg.dim)];
    y_hat = range_estimate(sat_poss, x_hat_);
    
    %Predicted covariance
    P_ = ekf.A*ekf.P*ekf.A' + ekf.Q;
    
    %%Update
    res = y - y_hat;                        %residual
    S = ekf.H*P_*ekf.H' + ekf.R;            %residual covariance
    K = P_*ekf.H' / S;                      %Kalman gain
    ekf.x_hat = x_hat_ + K*res;             %updated state estimate
    
    %updated covariance estimate
    ekf.P = (eye(ekf.cfg.n) - K*ekf.H)*P_*(eye(ekf.cfg.n) - K*ekf.H)' + K*ekf.R*K';
    
    %Symmetrize
    ekf.P = (ekf.P + ekf.P') ./ 2;
    
end
%}
function [ekf] = EKF_step(sat_poss, y, u, ekf)
    %%Validity checks
    
    %%Initial variables
    n_sat = size(y, 1);
    ekf.R = eye(n_sat);
    
    %Estimated range
    H = [ekf.h(ekf.x_hat, sat_poss, y), zeros(n_sat, ekf.cfg.n - ekf.cfg.dim)];
    y_hat = range_estimate(sat_poss, ekf.x_hat);
    
    
    %%Correction
    res = y - y_hat;                        %residual
    S = H*ekf.P*H' + ekf.R;                 %residual covariance
    K = ekf.P*H'*inv(S);                    %Kalman gain
    
    %updated covariance estimate
    P_ = (eye(ekf.cfg.n) - K*H)*ekf.P*(eye(ekf.cfg.n) - K*H)' + K*ekf.R*K';
    P_ = (P_ + P_') ./ 2;
    
    %%Predicton
    x_hat_ = ekf.x_hat + K*res;             %updated state estimate
    
    ekf.x_hat = ekf.A*x_hat_ + ekf.B*u;
    ekf.P = ekf.A*P_*ekf.A' + ekf.Q;     %Predicted covariance
    
end

%x_dot = Ax + Bu
%x = [p; v; theta; b_a; b_w; b_c; pseu(..)]';
%u = [a, w];