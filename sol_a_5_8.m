% solution to A5.8
clear all; close all; clc
% generate data
[t,y] = spline_data;
figure, plot(t,y); 
xlabel('t'), ylabel('y');
A = zeros(length(y),13);
G = A;
for ii = 1:length(t)
    [g,gp,gpp] = bsplines(t(ii));
    A(ii,:) = g';
    G(ii,:) = gpp';
end

cvx_begin
    variable x(13)
    minimize( norm(A*x-y, 2) )
    subject to
        G*x >= 0
cvx_end

% optimal spline is y_hat
y_hat = A*x;

hold on;
plot(t, y_hat, 'r');
legend('data','optimal spline');