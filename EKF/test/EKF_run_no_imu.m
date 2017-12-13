data = load('../../pyUblox/Satellite_data_x8.mat');
dr = load('pr_corr.mat');
%13 grader under loggtid

%data = load('Logs/Lab1-Data.mat');
ekf = EKF_init_no_imu();
n = size(data.pseudorange, 1);
%n = size(data.PR, 2);
%t = 1:n;


%BASE JUMP
base_end = 11969;
base_start = 1;%800; x8 begynner på iterasjon 32
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
%dr = zeros(32, n);
n_sats = zeros(1, n);

wgs84 = wgs84Ellipsoid('meters');

%Ekf step loop
P0 = [2799898.60635504; 479945.475254360; 5691590.57887127];
P0geo = [63.628611, 9.726939, 79];
c = 299792458.0;

for i = t
    j = i - base_start + 1;
    [pr, sat_poss] = parse_data(data, i, dr);

    p = ekf.x_hat(1:3);     %Estimated position
    [lat, lon, h] = ecef2geodetic(wgs84, P0(1), P0(2), P0(3));
    [el, azi] = satelazi(lat, lon, h, sat_poss);    
    [pr, sat_poss, el, azi] = elev_mask(pr, sat_poss, el, azi, 15);
    
    % EKF algorithm
    ekf.R = EKF_calculate_R(el)/8;
    ekf = EKF_step_no_imu(sat_poss, pr, zeros(5, 1), ekf);
    
    % For plotting
    pos(:, j)  = ekf.x_hat(1:3);
    vel(:, j)  = ekf.x_hat(4:6);
    bias(:, j) = ekf.x_hat(7);
    bias_dot(:, j) = ekf.x_hat(8);
    res(j) = ekf.res(end);
    
end

%figure(1); plot(t, pos); title('pos ecef'); xlabel('samples'), ylabel('position {m}')
%figure(3); plot(t, vel(100:end)); title('velocity'); xlabel('samples'), ylabel('velocity {m/s}')
hold on;
figure(1); plot(pos(1, 300:2000) - P0(1), pos(2, 300:2000) - P0(2), '*'); title('X8 position');

%Biases
figure(4); plot(t, bias); title('bias');
figure(5); plot(t, res);
%subplot(2, 1, 2); plot(t, bias_dot); title('bias_{dot}');

ekf.x_hat(1:3) - P0

%figure(6), histogram(res, 1);