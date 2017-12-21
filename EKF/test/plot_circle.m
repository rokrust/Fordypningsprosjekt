function plot_circle()
    hold on
    th = 0:pi/50:2*pi;
    xunit = 0.25 * cos(th);
    yunit = 0.25 * sin(th);
    h = plot(xunit, yunit, 'k');
    
    xunit = 0.5 * cos(th);
    yunit = 0.5 * sin(th);
    h = plot(xunit, yunit, 'k');
    
    xunit = 1 * cos(th);
    yunit = 1 * sin(th);
    h = plot(xunit, yunit, 'k');
    hold off
end