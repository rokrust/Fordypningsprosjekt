data = load('../../pyUblox/Satellite_data.mat');

[~, ~, n_sat] = parse_data(data, 1);
ekf = EKF_init_no_imu(n_sat);
n = size(data.pseudorange, 1);
n = 9000;
t = 1:n;

pos = zeros(3, n);
el = zeros(1, n);
el2 = zeros(1, n);
el3 = zeros(1, n);
el4 = zeros(1, n);
el5 = zeros(1, n);

for i = t
    if i == 0.8*10^4
        a = 3;
    end
    
    [pr, sat_poss, ~] = parse_data(data, i);
    ekf = EKF_step_no_imu(sat_poss, pr, zeros(5, 1), ekf); 
    pos(:, i) = ekf.x_hat(1:3);
    el(i) = satel(pos(:, i), sat_poss(:, 1));
    el2(i) = satel(pos(:, i), sat_poss(:, 2));
    el3(i) = satel(pos(:, i), sat_poss(:, 3));
    el4(i) = satel(pos(:, i), sat_poss(:, 4));
    el5(i) = satel(pos(:, i), sat_poss(:, 5));
end
plot(1:n, pos);
figure
plot(t, el, t, el2, t, el3, t, el4, t, el5);