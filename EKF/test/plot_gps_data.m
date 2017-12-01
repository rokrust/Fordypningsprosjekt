clear
load('x8_log201708251525.mat');
t = 1:size(nav.P0, 1);
%hold on;
[a, b] = hist(nav.sat, unique(nav.sat));