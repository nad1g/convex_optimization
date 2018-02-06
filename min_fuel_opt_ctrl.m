% Minimum fuel optimal control
% A3.17
A = [-1 .4 .8; 1 0 0; 0 1 0];
b = [1 0 .3]';
x_des = [7 2 -6]';
n = 3;  % state size
N = 30; % time horizon.

% direct method
cvx_begin
variable u(1,N)
variable x(n,N+1)
% trying 'user defined function'; can use sum(max(...)) directly!
minimize sum(fuel_use_map(u))
subject to
    x(:,2:N+1) == A*x(:,1:N)+b*u
    x(:,1) == zeros(3,1); % x(0) = 0
    x(:,N+1) == x_des % final state is = x_des
cvx_end;

figure, subplot(2,1,1), stairs(u,'b');
xlabel('time');
ylabel('actuator input');
legend('Direct method');

% LP
clear u;
H = zeros(3,N);
H(:,N) = b;
for k = N-1:-1:1
    H(:,k) = A*H(:,k+1);
end
cvx_begin
variables u(N) t(N)
minimize sum(t)
subject to
    H*u == x_des
    u <= t
    -u <= t
    2*u - 1 <= t
    -2*u -1 <= t
cvx_end

subplot(2,1,2), stairs(u,'r');
xlabel('time');
ylabel('actuator input');
legend('LP');
