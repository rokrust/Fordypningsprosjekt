clear data
data = load('../../pyUblox/Satellite_data_base.mat');
%13 grader under loggtid
%RTK begynner på 4590

%data = load('Logs/Lab1-Data.mat');
ekf = EKF_init_no_imu();
n = size(data.pseudorange, 1);

%BASE JUMP
base_end = 12000; %n
base_start = 1;%800; x8 begynner på iterasjon 32
t = base_start:base_end;
n = size(t, 2);

%Memory allocation
pos = zeros(3, n);
wgs84 = wgs84Ellipsoid('meters');
c = 299792458.0;

% True position in ECEF, NED and GEODETIC
P0 = [2799895.16266508;479943.493069408;5691588.52924916];
P0 = [2799893.47931252;479943.385189983;5691585.82531954];
P0 = [2799898.70162575;479945.262043602;5691591.39815845];
P0 = [2799898.536738521;479947.1649341192;5691587.415527507];
[lat_o, lon_o, h_o] = ecef2geodetic(wgs84, P0(1), P0(2), P0(3));
lat_o = 63.629156445; lon_o = 9.72687400; h_o = 1.2799810e+02;

% DGPS corrections
calculate_corrections(P0, data);

%{
%Ekf step loop
for i = t
    j = i - base_start + 1;
    [pr, sat_poss] = parse_base(data, i);
    
    % Mask out low elevation satellites
    p = ekf.x_hat(1:3);
    [el, azi] = satelazi(lat_o, lon_o, h_o, sat_poss);
    [pr, sat_poss, el, azi] = elev_mask(pr, sat_poss, el, azi, 15);
    
    di = ionospheric_correction(data.ionospheric, el, azi, lat_o, lon_o, data.t(i, 1));
    pr = pr - di'*c;
    
    % EKF algorithm
    ekf.R = EKF_calculate_R(el)/8;
    ekf = EKF_step_no_imu(sat_poss, pr, zeros(5, 1), ekf);    
    
    % For plotting
    [N, E, D] = ecef2ned(p(1), p(2), p(3), lat_o, lon_o, h_o, wgs84);
    pos(:, j) = ekf.x_hat(1:3);%[N; E; D];
end
%}
%hold on;
%figure(1); plot(pos(1, 800:10000), pos(2, 800:10000), '*'); title('X8 position');

%Biases
%figure(4); plot(t, bias); title('bias');
%figure(5); plot(t, res);
%subplot(2, 1, 2); plot(t, bias_dot); title('bias_{dot}');