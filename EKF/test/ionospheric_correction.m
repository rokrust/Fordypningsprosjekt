function [ I_gps ] = ionospheric_correction( iono_param, el, azi, lat, lon, t_gps )
%IONOSPHERIC_CORRECTION Summary of this function goes here
%   From http://www.navipedia.net/index.php/Klobuchar_Ionospheric_Model
a0 = iono_param(1); a1 = iono_param(2); a2 = iono_param(3); a3 = iono_param(4);
b0 = iono_param(5); b1 = iono_param(6); b2 = iono_param(7); b3 = iono_param(8);

%Convert to semi-circles
lat_u = deg2rad(lat) ./ pi;
lon_u = deg2rad(lon) ./ pi;

azi_r = deg2rad(azi) ./ pi;
el_r = deg2rad(el) ./ pi;

psi = 0.0137 ./ (el_r + 0.11) - 0.022;
lat_i = lat_u + psi .* cos(azi_r*pi);

%Saturate
ind = abs(lat_i) > 0.416;
lat_i(ind) = 0.416*sign(lat_i(ind));

lon_i = lon_u + psi.*sin(azi_r*pi)./cos(lat_i*pi);
lat_m = lat_i + 0.064.*cos((lon_i - 1.617)*pi);

%Time of intersection

t = 43200*lon_i + t_gps;
ind = t >= 86400;
while any(ind)
    t(ind) = t(ind) - 86400;
    ind = t >= 86400;
end

ind = t < 0;
while any(ind)
    t(ind) = t(ind) + 86400;
    ind = t < 0;
end

A_i = a0 + a1*lat_m + a2*(lat_m.^2) + a3*(lat_m.^3);  %Iono delay amplitude
P_i = b0 + b1*lat_m + b2*(lat_m.^2) + b3*(lat_m.^3);  %iono delay phase

%Saturate
ind = A_i < 0;
A_i(ind) = 0;

%Saturate
ind = P_i < 72000;
P_i(ind) = 72000;

X_i = 2*pi*(t - 50400) ./ P_i;
F = 1.0 + 16.0 .* (0.53 - el_r) .^ 3;

I_gps = 5*10^-9;
if abs(X_i) < 1.57
    I_gps = F .* (I_gps + A_i .* (1 - (X_i .^ 2)/2.0 + (X_i .^ 4)/24));
else
    I_gps = I_gps*F;
end

end

