function [ R ] = EKF_calculate_R( el )
    el = deg2rad(el);
    R = diag(sin(el).^-2);

end

