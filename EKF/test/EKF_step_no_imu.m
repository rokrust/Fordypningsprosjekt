function [ekf] = EKF_step_no_imu(sat_poss, y, u, ekf)
    
    %% INFO
    %In contrast, Kalman filters need to handle these jumps carefully or a 
    %large position jump will result. Innovation testing within the Kalman 
    %filter algorithm will easy identify these jumps. Unfortunately, 
    %innovation testing is usually performed on a per-satellite basis, 
    %so blindly apply such algorithms may result in you rejecting all of 
    %your measurements, usually for many consecutive epochs! You therefore 
    %need to handle the case where all (pseudorange) measurements exhibit 
    %the same jump between epochs and that the jump is close to an integer 
    %number of milliseconds.
    
    %% TODO
    %Implement innovation testing
    %Implement validity checks
    
    %% Initial variables
    n_sat = size(y, 1);
    ekf.R = 1*eye(n_sat);
    
    %Estimated range
    ekf.H = ekf.h(ekf.x_hat_, sat_poss, y);
    y_hat = range_estimate_no_imu(sat_poss, ekf.x_hat_);
    
    %% Validity checks
    %Observability check
    
    %% Update
    res = y - y_hat;                                                        %residual
    
    if res > 10^5
        ekf.x_hat_(end-1) = ekf.x_hat_(end-1) + 299792.458;
        ekf.H = ekf.h(ekf.x_hat_, sat_poss, y);
        y_hat = range_estimate_no_imu(sat_poss, ekf.x_hat_);
        res = y - y_hat;
    end
    
    S = ekf.H*ekf.P_*ekf.H' + ekf.R;                                        %residual covariance
    K = ekf.P_*ekf.H' / S;                                                  %Kalman gain
    ekf.x_hat = ekf.x_hat_ + K*res;                                         %updated state estimate
    
    ekf.res = res;
    
    ekf.P = (eye(ekf.cfg.n) - K*ekf.H)*ekf.P_*(eye(ekf.cfg.n) - K*ekf.H)' + K*ekf.R*K';
    %Symmetrize
    ekf.P = (ekf.P + ekf.P') ./ 2;
    
    %% Prediction
    ekf.x_hat_ = ekf.A*ekf.x_hat + ekf.B*u;
    ekf.P_ = ekf.A*ekf.P*ekf.A' + ekf.Q;    
end