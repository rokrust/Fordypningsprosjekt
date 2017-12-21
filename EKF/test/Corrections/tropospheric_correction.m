function d_t = tropospheric_correction(el, lat, lon, h, temp0)
    lat_r = deg2rad(lat); lon_r = deg2rad(lat);
    el_r = deg2rad(el);
    humidity = 0.001;
    
    if h < -100.0 || 1e4 < h
        d_t = 0.0;
    end
    
    if h < 0
        h = 0;
    end
    
    pres = 1013.25*(1.0-2.2557e-5*h)^5.2568;
    temp = temp0 - 6.5e-3*h+273.16;
    e = 6.108*humidity*exp((17.15*temp-4684.0)/(temp-38.45));
    
    % saastamoninen model
    z = pi / 2.0 - el_r;
    trph = (0.0022768 * pres /(1.0-0.00266*cos(2.0*lat_r)-0.00028*h/1e3))./cos(z);
    trpw = (0.002277  * (1255.0 / temp + 0.05) * e) ./ cos(z);
    
    d_t = trph+trpw;
end