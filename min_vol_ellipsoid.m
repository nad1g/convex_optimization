% minimum volume ellipsoid (2d example)
% create some random points an plot them
clear all; close all; clc;
a = randi(5,[10 2]).*(rand(10,2) - .5);
figure, plot(a(:,1), a(:,2),'o','LineWidth',2);
grid on;
axis tight;

%
% note: 
% an ellipsoid (at origin) can be represented as E = {x|x'Px <= 1}
% volume of an ellipsoid (in 2d, just area) is prop.to. prod of semi-axes
% semi-axes = sqrt(of the eigenvalues of P^-1)
% volume prop. to. prod(sqrt(eig(P^-1))
% prod of eigenvalues of P^-1 = det P^-1. so, it follows that
% vol of ellipsoid prop. to -1/2 * log (det P).

% find an ellipsoid that covers these points, i.e., 
% constraint: a_i'Pa_i <= 1.

cvx_begin
variable P(2,2) symmetric semidefinite
minimize -log_det(P)
subject to
a*P*a' <= 1
cvx_end

% plot the ellipse
x = [cos(0:pi/100:2*pi) ; sin(0:pi/100:2*pi)];
y = inv(P)*x;
hold on;
plot(y(1,:),y(2,:),'r','LineWidth',1.5);