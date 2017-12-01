data = load('../../pyUblox/Satellite_data.mat');

[~, ~, n_sat] = parse_data(data, 1);
ekf = EKF_init_no_imu(n_sat);
n = size(data.pseudorange, 1);
%n = 9000;
t = 1:n;

pos = zeros(3, n);
vel = zeros(3, n);
bias = zeros(1, n);

for i = t
    [pr, sat_poss, ~] = parse_data(data, i);
    ekf = EKF_step_no_imu(sat_poss, pr, zeros(5, 1), ekf); 
    pos(:, i)  = ekf.x_hat(1:3);
    vel(:, i)  = ekf.x_hat(4:6);
    bias(:, i) = ekf.x_hat(7);
    
    if i > 200
        if bias(1, i) - bias(1, i-5) > 2000
            a = 3;
        end
    end
    
end
plot(t, data.pseudorange(1:end, 32), t, data.pseudorange(1:end, 8));
%figure
%plot(t, vel);
%figure
%plot(t, bias);

