data = load('../../pyUblox/Satellite_data_base.mat');
%13 grader under loggtid

%data = load('Logs/Lab1-Data.mat');
[~, ~, n_sat] = parse_data(data, 1);    %Find initial number of satellites
ekf = EKF_init_no_imu(n_sat);
n = size(data.pseudorange, 1);
%n = size(data.PR, 2);
%t = 1:n;


%BASE JUMP
base_end = n;%10000;
base_start = 1;%800;
t = base_start:base_end;
n = size(t, 2);
%t = 1:n;

%Memory allocation
pos = zeros(3, n);
llh = zeros(3, n);
vel = zeros(3, n);
bias = zeros(1, n);
bias_dot = zeros(1, n);
res = zeros(1, n);
n_sats = zeros(1, n);

wgs84 = wgs84Ellipsoid('meters');

%Ekf step loop
P0 = [2799897.28828497;479945.270743682;5691587.69210791];
P0geo = [63.628611, 9.726939, 79];
c = 299792458.0;

for i = t
    j = i - base_start + 1;
    [pr, sat_poss, ~] = parse_data(data, i);

    p = ekf.x_hat(1:3);     %Estimated position
    [lat, lon, h] = ecef2geodetic(wgs84, p(1), p(2), p(3));
    [el, azi] = satelazi(lat, lon, h, sat_poss);
        
    %Remove all satellites with elevation less than 10 deg,
    %unless all are have 0 deg elevation    
    
    if j == 648
    end
    
    if sum(el > 15) >= 4
        ind = el < 15;
        el(ind) = [];
        azi(ind) = [];
        pr(ind) = [];
        sat_poss(:, ind) = [];
        
        % Corrections
        di = ionospheric_correction(data.ionospheric, el, azi, lat, lon, data.t(i));
        ds = sagnac_correction(p, sat_poss);
        pr = pr - di'*c + ds'*c;
    end
    n_sats(i) = size(el, 2);
    %ekf.R = EKF_calculate_R(el)/8;
    
    % EKF algorithm
    ekf = EKF_step_no_imu(sat_poss, pr, zeros(5, 1), ekf);
    
    % For plotting
    llh(:, j)  = [lat; lon; h];
    pos(:, j)  = ekf.x_hat(1:3);
    vel(:, j)  = ekf.x_hat(4:6);
    bias(:, j) = ekf.x_hat(7);
    bias_dot(:, j) = ekf.x_hat(8);
    res(j) = ekf.res(end);
    
end

%figure(1); plot(t, pos); title('pos ecef'); xlabel('samples'), ylabel('position {m}')
%figure(2); subplot(211); plot(t, llh(1, :));subplot(212); plot(t, llh(2, :));
%figure(2); subplot(211); plot(t(100:end), llh(1, 100:end));title('lat');subplot(212); plot(t(100:end), llh(2, 100:end));title('lon')
%figure(3); plot(t, vel(100:end)); title('velocity'); xlabel('samples'), ylabel('velocity {m/s}')
hold on;
P1 = mean(pos(:, 800:10000)');
figure(1); plot(pos(1, 100:10000)-P1(1), pos(2, 100:10000)-P1(2), '*');

%Biases
figure(4);
plot(t, bias); title('bias');
figure(5); plot(t, res);
%subplot(2, 1, 2); plot(t, bias_dot); title('bias_{dot}');

ekf.x_hat(1:3) - P0

%figure(6), histogram(res, 1);
