clc;clear;close all;
path = ginput() * 100.0;

n_order       = 7;% order of poly
n_seg         = size(path,1)-1;% segment number
n_poly_perseg = (n_order+1); % coef number of perseg

ts = zeros(n_seg, 1);
% calculate time distribution in proportion to distance between 2 points
dist     = zeros(n_seg, 1);
dist_sum = 0;
T        = 25;
t_sum    = 0;

%calculate each path's distance 
for i = 1:n_seg
    dist(i) = sqrt((path(i+1, 1)-path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2);
    dist_sum = dist_sum+dist(i);
end

%calculate each path's time in proportion to distance between 2 points 
% for i = 1:n_seg-1
%     ts(i) = dist(i)/dist_sum*T;
%     t_sum = t_sum+ts(i);
% end
% ts(n_seg) = T - t_sum;

%calculate each path's time use trapezoidal velocity  
v_max = 10.0;
a_max = 5.0;
s_desized_min = v_max^2 / a_max;
for i = 1:n_seg
    if(dist(i) >= s_desized_min)
        ts(i) = 2 * v_max/a_max + (dist(i) - s_desized_min) / v_max;
    else
        ts(i) = 2 * sqrt(dist(i)/a_max);
    end
end

% or you can simply set all time distribution as 1
% for i = 1:n_seg
%     ts(i) = 1.0;
% end

poly_coef_x = MinimumSnapQPSolver(path(:, 1), ts, n_seg, n_order);
poly_coef_y = MinimumSnapQPSolver(path(:, 2), ts, n_seg, n_order);


% display the trajectory
X_n = [];
Vx_n = [];
Accx_n = [];
Jerkx_n = [];
Y_n = [];
Vy_n = [];
Accy_n = [];
Jerky_n = [];
t_axis = [];
k = 1;
tstep = 0.01;
for i=0:n_seg-1
    %#####################################################
    % STEP 3: get the coefficients of i-th segment of both x-axis
    % and y-axis
    Pxi = poly_coef_x(i * 8 + 1:(i+1)*8);
    Pxi = flipud(Pxi); 
    Pyi = poly_coef_y(i * 8 + 1:(i+1)*8);
    Pyi = flipud(Pyi); 
    for t = 0:tstep:ts(i+1)
        X_n(k)  = PolyValue(Pxi, t, 0);
        Vx_n(k)  = PolyValue(Pxi, t, 1);
        Accx_n(k)  = PolyValue(Pxi, t, 2);
        Jerkx_n(k) = PolyValue(Pxi, t, 3);
        Y_n(k)  = PolyValue(Pyi, t, 0);
        Vy_n(k)  = PolyValue(Pyi, t, 1);
        Accy_n(k)  = PolyValue(Pyi, t, 2);
        Jerky_n(k) = PolyValue(Pyi, t, 3);
        t_axis(k) = k * tstep;
        k = k + 1;
    end
end

plot(X_n, Y_n , 'Color', [0 1.0 0], 'LineWidth', 2);
hold on;grid on;
scatter(path(1:size(path, 1), 1), path(1:size(path, 1), 2));
title('Minimum Snap Trajectory');

figure;
subplot(2,2,1);
plot(t_axis, X_n,'r','LineWidth',2);hold on;
plot(t_axis, Y_n,'g','LineWidth',2);hold on;grid on;
legend('X-Pos','Y-Pos');
title('Minimum Snap Trajectory Position Result');

subplot(2,2,3);
plot(t_axis, Vx_n,'b','LineWidth',2);hold on;
plot(t_axis, Vy_n,'c','LineWidth',2);hold on;grid on;
legend('X-Vel','Y-Vel');
title('Minimum Snap Trajectory Velocity Result');

subplot(2,2,2);
plot(t_axis, Accx_n,'m','LineWidth',2);hold on;
plot(t_axis, Accy_n,'y','LineWidth',2);hold on;grid on;
legend('X-Acc','Y-Acc');
title('Minimum Snap Trajectory Acceleration Result');

subplot(2,2,4);
plot(t_axis, Jerkx_n,'k','LineWidth',2);hold on;
plot(t_axis, Jerky_n,'y','LineWidth',2);hold on;grid on;
legend('X-Jerk','Y-Jerk');
title('Minimum Snap Trajectory Jerk Result');