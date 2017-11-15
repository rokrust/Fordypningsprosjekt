function [ekf] = EKF_step_no_imu(sat_poss, y, u, ekf)
    %%Validity checks
    
    %%Initial variables
    n_sat = size(y, 1);
    ekf.R = eye(n_sat);
    
    %Estimated range
    H = ekf.h(ekf.x_hat, sat_poss, y);
    y_hat = range_estimate_no_imu(sat_poss, ekf.x_hat);
    
    
    %%Correction
    res = y - y_hat;                        %residual
    S = H*ekf.P*H' + ekf.R;         %residual covariance
    K = ekf.P*H'*inv(S);                %Kalman gain
    
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