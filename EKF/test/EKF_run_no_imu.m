%% Load mat files
data = load('../../pyUblox/Satellite_data_x8.mat');
dr = load('pr_corr.mat');
%dr = dr.dr;
P0 = load('P0.mat');
P0 = P0.P0;
p = genpath('.');
addpath(p);
%13 grader under loggtid

use_DGPS = false;%true;
ekf = EKF_init_no_imu();
n = size(data.pseudorange, 1);

base_end = n;
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
c = 299792458.0;

acc_tot = zeros(3, 717);

%% Ekf step loop
[lat_o, lon_o, h_o] = ecef2geodetic(wgs84, P0(1), P0(2), P0(3));
lat_o = rad2deg(mean(GpsFixRtk.base_lat)); 
lon_o = rad2deg(mean(GpsFixRtk.base_lon)); 
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
    [pr, sat_poss, el, azi] = elev_mask(pr, sat_poss, el, azi, 10); 
    [N, E, D] = ecef2ned(p(1), p(2), p(3), lat_o, lon_o, h_o, wgs84);
    
    if use_DGPS == false
        di = ionospheric_correction(data.ionospheric, el, azi, lat_o, lon_o, data.t(i));
        ds = sagnac_correction(p, sat_poss);
        dt = tropospheric_correction(el, lat_o, lon_o, h_o, 15);
        dt = 0;
        %di = 0;
        pr = pr - ds' - di'*c - dt';
    end
    
    % Match acceleration measurements in time with GPS
    acc = zeros(3, 1);
    if i >= 4590 && i <= 4590+717
        k = i - 4590+1;
        prev_time_diff = Inf;
        for ii = l:717
            curr_time_diff = abs(t_acc(l)-t_rtk(k));
            if curr_time_diff > prev_time_diff
                %prev_time_diff
                break;
            end
            
            l = l + 1;
            prev_time_diff = curr_time_diff;
        
        end
        acc = [Acceleration.x(l-1); 
               Acceleration.y(l-1); 
               Acceleration.z(l-1)];
        [acc(1),acc(2),acc(3)] = ned2ecefv(acc(1),acc(2),acc(3),lat_o,lon_o);
        acc_tot(:, k) = acc;
    end
    acc = zeros(3, 1);
    
    % EKF algorithm step
    ekf.R = EKF_calculate_R(el)/16;
    ekf = EKF_step_no_imu(sat_poss, pr, acc, ekf);
    
    % For plotting
    pos(:, j)  = [N; E; D];
    vel(:, j)  = ekf.x_hat(4:6);
    bias(j) = ekf.x_hat(7);
    bias_dot(j) = ekf.x_hat(8);
    res(j) = ekf.res(end);
    
end
RTKlib_pos = [GpsFixRtk.n, GpsFixRtk.e, GpsFixRtk.d];
my_pos2 = pos(:, 4590:4590+716)';
rms_vec = rms(RTKlib_pos(idx, :)-my_pos2(idx, :))

%% Plot data
idx = GpsFixRtk.src == 11266;
t_plot = data.t(4590:4590+716);
idx(1:50) = 0;

%figure(2); plot(GpsFixRtk.n, GpsFixRtk.e, '*'); 
%idx = GpsFixRtk.src == 11266;
%hold on; figure(2); plot(GpsFixRtk.n(idx), GpsFixRtk.e(idx))
%hold on; figure(2); plot(pos(1, 4590:4590+707), pos(2, 4590:4590+707), '*'); title('X8 position');
%hold on; figure(1); plot(pos(1, 100:1000), pos(2, 100:1000), '*'); title('X8 position');

%figure(1); subplot(3, 1, 1); plot(t_plot(idx)-t_plot(51), GpsFixRtk.n(idx)); hold on; plot(t_plot(idx)-t_plot(51), my_pos2(idx, 1))
%figure(1); subplot(3, 1, 2); plot(t_plot(idx)-t_plot(51), GpsFixRtk.e(idx)); hold on; plot(t_plot(idx)-t_plot(51), my_pos2(idx, 2))
%figure(1); subplot(3, 1, 3); plot(t_plot(idx)-t_plot(51), GpsFixRtk.d(idx)); hold on; plot(t_plot(idx)-t_plot(51), my_pos2(idx, 3))

hold on; plot(t_plot(idx)-t_plot(51), my_pos2(idx, 1))
hold on; plot(t_plot(idx)-t_plot(51), my_pos2(idx, 2))
hold on; plot(t_plot(idx)-t_plot(51), my_pos2(idx, 3))

%Biases
%figure(4); hold off; subplot(3, 1, 2); plot(t, bias); title('Standard GPS X8 bias'); xlabel('Samples'); ylabel('Bias [m]');
%figure(5); plot(t, res);
%subplot(2, 1, 2); plot(t, bias_dot); title('bias_{dot}');
