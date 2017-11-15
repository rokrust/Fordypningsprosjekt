data = load('Lab1-Data.mat');

[~, ~, n_sat] = parse_data(data, 1);
ekf = EKF_init_no_imu(n_sat);
n = size(data.Tow, 2);

pos = zeros(3, n);
for i = 1:n
    [pr, sat_poss, ~] = parse_data(data, i);
    ekf = EKF_step_no_imu(sat_poss, pr, zeros(5, 1), ekf); 
    pos(:, i) = ekf.x_hat(1:3);
end
plot(1:n, pos);