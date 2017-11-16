data = load('Lab1-Data.mat');

[~, ~, n_sat] = parse_data(data, 1);
ekf = EKF_init(n_sat);
n = size(data.Tow, 2);

pos = zeros(3, n);
for i = 1:n
    [pr, sat_poss, ~] = parse_data(data, i);
    ekf = EKF_step(sat_poss, pr, zeros(6, 1), ekf); 
    pos(:, i) = ekf.x_hat_(1:3);
    pos(:,i)
end
%%
figure
plot(1:n, pos);