%data = load('../../pyUblox/Satellite_data_base.mat');

data = load('Logs/Lab1-Data.mat');
[~, ~, n_sat] = parse_data(data, 1);    %Find initial number of satellites
ekf = EKF_init_no_imu(n_sat);
%n = size(data.pseudorange, 1);
n = size(data.PR, 2);
t = 1:n;
%{
%BASE JUMP
base_end = 11240;
base_start = 800;
t = base_start:base_end;
n = size(t, 2);
%t = 1:n;
%}

%Memory allocation
pos = zeros(3, n);
llh = zeros(3, n);
vel = zeros(3, n);
bias = zeros(1, n);
bias_dot = zeros(1, n);

wgs84 = wgs84Ellipsoid('meters');

%Ekf step loop
P0 = [2.799832e+06; 4.79942e+05; 5.691477e+06];
P0geo = [63.628611, 9.726939, 79];
c = 299792458.0;
for i = t
    j = i;% - base_start + 1;
    [pr, sat_poss, ~] = parse_data(data, i);
    
    % Corrections
    p = ekf.x_hat(1:3);     %Estimated position
    [lat, lon, h] = ecef2geodetic(wgs84, p(1), p(2), p(3));
    [el, azi] = satelazi(lat, lon, h, sat_poss);
    
    %Remove all satellites with elevation less than 10 deg,
    %unless all are have 0 deg elevation
    if any(el)
        ind = el < 5;
        el(ind) = [];
        azi(ind) = [];
        pr(ind) = [];
        sat_poss(:, ind) = [];
    
        %di = ionospheric_correction(data.ionospheric, el, azi, lat, lon, data.t(i));
        %pr = pr - di'*c;
    end
    
    % EKF algorithm
    ekf = EKF_step_no_imu(sat_poss, pr, zeros(5, 1), ekf);
    
    % For plotting
    llh(:, j)  = [lat; lon; h];
    pos(:, j)  = ekf.x_hat(1:3);
    vel(:, j)  = ekf.x_hat(4:6);
    bias(:, j) = ekf.x_hat(7);
    bias_dot(:, j) = ekf.x_hat(8);
    
end

%% Plot satellite positions
% Sat in red and actual position in blue
%{
ind = [1, 4, 8, 11, 14, 32];
for i = t
    for j = ind
        p = reshape(data.satPos(i, j, :), [3, 1]);
        plot3(p(1), p(2), p(3), '*r');
        axis(10^7*[-1 3 -2 2 -1 2.5]);
        hold on;
    end
    plot3(P0(1)/10^7, P0(2)/10^7, P0(3)/ 10^7, 'b*');
    hold off;
    drawnow
    pause(0.0001)
end
%}

figure; plot(t, pos); title('pos ecef'); xlabel('samples'), ylabel('position {m}')
figure; plot(t, llh(1:2, :)); title('lat,lon');
figure; plot(t, vel); title('velocity'); xlabel('samples'), ylabel('velocity {m/s}')

%Biases
figure;
plot(t, bias); title('bias');
%subplot(2, 1, 2); plot(t, bias_dot); title('bias_{dot}');

ekf.x_hat(1:3) - data.P0
