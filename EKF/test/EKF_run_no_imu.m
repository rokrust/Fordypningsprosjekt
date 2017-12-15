data = load('../../pyUblox/Satellite_data_x8.mat');
dr = load('pr_corr.mat');
P0 = load('P0.mat');
P0 = P0.P0;
%13 grader under loggtid

ekf = EKF_init_no_imu();
n = size(data.pseudorange, 1);

base_end = 4590+716;%11969;
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

%Ekf step loop
%P0 = [2799898.70162591;479945.262043493;5691591.39815204];
[lat_o, lon_o, h_o] = ecef2geodetic(wgs84, P0(1), P0(2), P0(3));
lat_o = 63.629156445; lon_o = 9.72687400; h_o = 1.2799810e+02;

for i = t
    j = i - base_start + 1;
    [pr, sat_poss] = parse_data(data, i, dr);

    p = ekf.x_hat(1:3);     %Estimated position

    % Mask out low elevation satellites
    [el, azi] = satelazi(lat_o, lon_o, h_o, sat_poss);
    [pr, sat_poss, el, azi] = elev_mask(pr, sat_poss, el, azi, 5); 
    [N, E, D] = ecef2ned(p(1), p(2), p(3), lat_o, lon_o, h_o, wgs84);
    %di = ionospheric_correction(data.ionospheric, el, azi, lat_o, lon_o, data.t(i, 1));
    %pr = pr - di'*c;

    % EKF algorithm step
    ekf.R = EKF_calculate_R(el)/24;
    ekf = EKF_step_no_imu(sat_poss, pr, 0, ekf);
    
    % For plotting
    pos(:, j)  = [N; E; D];
    vel(:, j)  = ekf.x_hat(4:6);
    bias(:, j) = ekf.x_hat(7);
    bias_dot(:, j) = ekf.x_hat(8);
    res(j) = ekf.res(end);
    
end
hold on;
figure(1); plot(GpsFixRtk.n, GpsFixRtk.e, '*'); 
%figure(1); plot(pos(1, 4590:4590+716));
figure(1); plot(pos(1, 4590:4590+716), pos(2, 4590:4590+716), '*'); title('X8 position');
%hold on; figure(1); plot(pos(1, 100:1000), pos(2, 100:1000), '*'); title('X8 position');

%Biases
%figure(4); hold off; subplot(3, 1, 2); plot(t, bias); title('Standard GPS X8 bias'); xlabel('Samples'); ylabel('Bias [m]');
%figure(5); plot(t, res);
%subplot(2, 1, 2); plot(t, bias_dot); title('bias_{dot}');
