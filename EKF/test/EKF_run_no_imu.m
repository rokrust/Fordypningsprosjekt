%% Load mat files
data = load('../pyUblox/Satellite_data_x8.mat');
dr = load('pr_corr.mat');

P0 = load('P0.mat');
P0 = P0.P0;
p = genpath('.');
addpath(p);

use_DGPS = true;

ekf = EKF_init_no_imu();
n = size(data.pseudorange, 1);

% This is changed if a subset of the data set is to be used
base_end = n;
base_start = 1;
t = base_start:base_end;
n = size(t, 2);

%Memory allocation
pos = zeros(3, n);
llh = zeros(3, n);
vel = zeros(3, n);
bias = zeros(1, n);
bias_dot = zeros(1, n);

wgs84 = wgs84Ellipsoid('meters');
c = 299792458.0;

%% Ekf step loop
P0 = [2799898.70162591;479945.262043493;5691591.39815204];
[lat_o, lon_o, h_o] = ecef2geodetic(wgs84, P0(1), P0(2), P0(3));
h_o = mean(GpsFixRtk.base_height);

l = 15;
load('Logs/Acceleration.mat');
idy = Acceleration.src_ent ~= 27;
Acceleration.x(idy) = [];
Acceleration.y(idy) = [];
Acceleration.z(idy) = [];
Acceleration.timestamp(idy) = [];
t_acc = Acceleration.timestamp;
t_rtk = GpsFixRtk.timestamp;

for i = t
    j = i - base_start + 1;
    [pr, sat_poss] = parse_data(data, i, dr, use_DGPS, ekf.x_hat(end-1));

    p = ekf.x_hat(1:3);     %Estimated position

    % Mask out low elevation satellites
    [el, azi] = satelazi(lat_o, lon_o, h_o, sat_poss);
    [pr, sat_poss, el, azi] = elev_mask(pr, sat_poss, el, azi, 15); 
    [N, E, D] = ecef2ned(p(1), p(2), p(3), lat_o, lon_o, h_o, wgs84);
    
    %Apply corrections only for standard GPS
    if use_DGPS == false
        di = ionospheric_correction(data.ionospheric, el, azi, lat_o, lon_o, data.t(i));
        ds = sagnac_correction(p, sat_poss);
        dt = tropospheric_correction(el, lat_o, lon_o, h_o, 15);
        pr = pr - ds' - di'*c - dt';
    end
    
    % EKF algorithm step
    ekf.R = EKF_calculate_R(el)/16;
    ekf = EKF_step_no_imu(sat_poss, pr, zeros(3, 1), ekf);
    
    % For plotting
    pos(:, j)  = [N; E; D];
    vel(:, j)  = ekf.x_hat(4:6);
    bias(j) = ekf.x_hat(7);
    bias_dot(j) = ekf.x_hat(8);    
end