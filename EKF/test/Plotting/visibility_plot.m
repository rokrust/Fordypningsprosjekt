function visibility_plot( el, azi )
    pax = polaraxes;
    polarplot(deg2rad(azi), el, 'o')
    pax.ThetaDir = 'counterclockwise';
    pax.ThetaZeroLocation = 'top';
    pax.RDir = 'reverse';
    pax.RLim = [0 90];
    rticks(pax, 10:10:80);
    rtickformat('%g\x00B0')
    rticklabels({'', '20' , '', '40', '', '60' , '', '80'})
    thetaticks(0:22.5:360)
    thetaticklabels({'N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW'})
end

