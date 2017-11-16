function [ekf] = EKF_step(sat_poss, y, u, ekf)
   
    
    %% Initial variables
    n_sat = size(y, 1);
    ekf.R = eye(n_sat);
        %Estimated range
    ekf.H = [ekf.h(ekf.x_hat_, sat_poss, y), zeros(n_sat, ekf.cfg.n - ekf.cfg.dim)];
    y_hat = range_estimate(sat_poss, ekf.x_hat_);

    %% Validity checks

    
    %% Update
    res = y - y_hat;                                                        %residual
    S = ekf.H*ekf.P_*ekf.H' + ekf.R;                                        %residual covariance
    K = ekf.P_*ekf.H' / S;                                                  %Kalman gain
    ekf.x_hat = ekf.x_hat_ + K*res;                                         %updated state estimate
    ekf.P = (eye(ekf.cfg.n) - K*ekf.H)*ekf.P_*(eye(ekf.cfg.n) - K*ekf.H)' + K*ekf.R*K';
    %Symmetrize
    ekf.P = (ekf.P + ekf.P') ./ 2;
    
    %% Prediction
    ekf.x_hat_ = ekf.A*ekf.x_hat + ekf.B*u;
    ekf.P_ = ekf.A*ekf.P*ekf.A' + ekf.Q;    
end

%x_dot = Ax + Bu
%x = [p; v; theta; b_a; b_w; b_c; pseu(..)]';
%u = [a, w];