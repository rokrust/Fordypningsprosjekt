data = load('../../pyUblox/Satellite_data.mat');

[~, ~, n_sat] = parse_data(data, 1);
ekf = EKF_init_no_imu(n_sat);
n = size(data.pseudorange, 1);
%n = 9000;
t = 1:n;

pos = zeros(3, n);
vel = zeros(3, n);
bias = zeros(1, n);
wgs84 = wgs84Ellipsoid('meters');
for i = t
    pos(:, i)  = ekf.x_hat(1:3);
    [pr, sat_poss, ~] = parse_data(data, i);
    
    p = ekf.x_hat(1:3);
    [lat, lon, h] = ecef2geodetic(wgs84, p(1), p(2), p(3));
    [el, azi] = satelazi(p, sat_poss);
    
    
    di = ionospheric_correction(data.ionospheric, el, azi, lat, lon, data.t(i));
    %pr = pr - di;
    
    ekf = EKF_step_no_imu(sat_poss, pr, zeros(5, 1), ekf); 
    
    %[lat, lon, h] = ecef2geodetic(wgs84, p(1), p(2), p(3));
    %pos(:, i)  = [lat, lon, h];%ekf.x_hat(1:3);
    vel(:, i)  = ekf.x_hat(4:6);
    bias(:, i) = ekf.x_hat(7);
    
end
%{
for i = t
    plot3(pos(1, i), pos(2, i), pos(3, i), '*r');
    drawnow
    pause(0.1)
end
%}
plot(t, data.pseudorange(1:end, 11)/10^6, '*r');
%plot(t, data.pseudorange(1:end, 32), t, data.pseudorange(1:end, 8));
%figure
%plot(t, vel);
%figure
%plot(t, bias);

%62.2108
%65.2442

%7.3705
%13.6751
