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
dr = zeros(32, n);
wgs84 = wgs84Ellipsoid('meters');

P0 = [2799898.60635504; 479945.475254360; 5691590.57887127];
c = 299792458.0;

%Ekf step loop
for i = t
    j = i - base_start + 1;
    [pr, sat_poss] = parse_base(data, i);
    
    % Mask out low elevation satellites
    p = ekf.x_hat(1:3);
    [lat, lon, h] = ecef2geodetic(wgs84, p(1), p(2), p(3));
    [el, azi] = satelazi(lat, lon, h, sat_poss);
    [pr, sat_poss, el, azi] = elev_mask(pr, sat_poss, el, azi, 15);
    
    % Calculate DGPS corrections
    dr(:, i) = calculate_corrections(P0, data, ekf.x_hat(7), i);
    
    % EKF algorithm
    ekf.R = EKF_calculate_R(el)/8;
    ekf = EKF_step_no_imu(sat_poss, pr, zeros(5, 1), ekf);    
    
    % For plotting
    pos(:, j) = ekf.x_hat(1:3);
end
hold on;
figure(1);  plot(pos(1, 100:end) - P0(1), pos(2, 100:end)-P0(2), '*'); title('Base position');
dr(:, 1:31) = [];
save('pr_corr.mat', 'dr')