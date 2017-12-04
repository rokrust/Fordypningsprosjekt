data = load('../../pyUblox/Satellite_data_base.mat');

[~, ~, n_sat] = parse_data(data, 1);    %Find initial number of satellites
ekf = EKF_init_no_imu(n_sat);
n = size(data.pseudorange, 1);

%BASE JUMP
base_end = 11240;
base_start = 800;
t = base_start:base_end;
n = size(t, 2);
%t = 1:n;

%Memory allocation
pos = zeros(3, n);
llh = zeros(3, n);
vel = zeros(3, n);
bias = zeros(1, n);
bias_dot = zeros(1, n);
wgs84 = wgs84Ellipsoid('meters');

%Ekf step loop
P0 = [2.799832e+06; 4.79942e+05; 5.691477e+06];
for i = t
    j = i - base_start + 1;
    [pr, sat_poss, ~] = parse_data(data, i);
    
    p = ekf.x_hat(1:3);     %Estimated position
    [lat, lon, h] = ecef2geodetic(wgs84, p(1), p(2), p(3));
    [el, azi] = satelazi(p, sat_poss);
    
    ind = el < 10;
    el(ind) = [];
    azi(ind) = [];
    pr(ind) = [];
    sat_poss(:, ind) = [];
    
    %di = ionospheric_correction(data.ionospheric, el, azi, lat, lon, data.t(i));
    %pr = pr - di;
    
    ekf = EKF_step_no_imu(sat_poss, pr, zeros(5, 1), ekf);
    llh(:, j)  = [lat; lon; h];
    pos(:, j)  = ekf.x_hat(1:3);
    vel(:, j)  = ekf.x_hat(4:6);
    bias(:, j) = ekf.x_hat(7);
    bias_dot(:, j) = ekf.x_hat(8);
    
end
%{
for i = t
    plot3(pos(1, i), pos(2, i), pos(3, i), '*r');
    drawnow
    pause(0.1)
end
%}
%plot(t, pos, '-');
%plot(t, data.pseudorange(1:end, 11)/10^6, '*r');

plot(t, llh(1:2, :));