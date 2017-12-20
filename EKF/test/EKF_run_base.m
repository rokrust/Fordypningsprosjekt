clear data
data = load('../../pyUblox/Satellite_data_base.mat');
p = genpath('.');
addpath(p);
%13 grader under loggtid
%RTK begynner på 4590

%data = load('Logs/Lab1-Data.mat');
ekf = EKF_init_no_imu();
n = size(data.pseudorange, 1);

%BASE JUMP
base_end = n;%13000; %n
base_start = 1;%800; x8 begynner på iterasjon 32
t = base_start:base_end;
n = size(t, 2);

%Memory allocation
pos = zeros(3, n);
pos_ecef = zeros(3, n);
ion = zeros(1, n);
sagnac = zeros(1, n);
elev = zeros(1, n);
bias = zeros(1, n);
wgs84 = wgs84Ellipsoid('meters');
c = 299792458.0;

% True position in ECEF, NED and GEODETIC
P0 = [2799898.70162591;479945.262043493;5691591.39815204];
%P0 = [2799880.3;       479946.09;       5691631];

[lat_o, lon_o, h_o] = ecef2geodetic(wgs84, P0(1), P0(2), P0(3));

% DGPS corrections
calculate_corrections(P0, data);

%Ekf step loop
for i = t
    j = i - base_start + 1;
    [pr, sat_poss] = parse_base(data, i);
    
    % Mask out low elevation satellites
    p = ekf.x_hat(1:3);
    [el, azi] = satelazi(lat_o, lon_o, h_o, sat_poss);
    if i > 30
        elev(j-30) = el(8);
    end
    [pr, sat_poss, el, azi] = elev_mask(pr, sat_poss, el, azi, 15);
    
    di = ionospheric_correction(data.ionospheric, el, azi, lat_o, lon_o, data.t(i));
    ds = sagnac_correction(p, sat_poss);
    dt = tropospheric_correction(el, lat_o, lon_o, h_o, 10);
    pr = pr - ds' + di'*c + dt';
    
    % EKF algorithm
    ekf.R = EKF_calculate_R(el)/16;
    ekf = EKF_step_no_imu(sat_poss, pr, zeros(3, 1), ekf);    
    
    % For plotting
    [N, E, D] = ecef2ned(p(1), p(2), p(3), lat_o, lon_o, h_o, wgs84);
    pos(:, j) = [N; E; D];
    bias(j) = ekf.x_hat(end-1);
    pos_ecef(:, j) = ekf.x_hat(1:3);
end

%hold on; plot(pos(1, 800:10000), pos(2, 800:10000), '*')

%t_corr = data.t(31:end)+bias/c;

dr = load('pr_corr.mat');
%dr.dr(:, 1:30) = [];
dr.dr = dr.dr + bias - data.sv_clock(31:end, :)'*c;
for i = 100:size(dr.dr, 2)
    ind = abs(dr.dr(:, i) - dr.dr(:, i-1)) > 20;
    dr.dr(ind, i) = dr.dr(ind, i-1);
end
%plot(t_corr - t_corr(1), dr.dr(22, :))
%dr = dr.dr;
save('pr_corr.mat', 'dr', 't_corr')


P0 = mean(pos_ecef(:, 800:10000)')'
[lat_o, lon_o, h_o] = ecef2geodetic(wgs84, P0(1), P0(2), P0(3));
[x, y, z] = geodetic2ecef(wgs84, GpsFixRtk.base_lat, GpsFixRtk.base_lon, GpsFixRtk.base_height, 'radians');
p_err = [deg2rad(lat_o) - GpsFixRtk.base_lat(1); deg2rad(lon_o) - GpsFixRtk.base_lon(1); h_o - GpsFixRtk.base_height(1)];
save('P0.mat', 'P0');