% graphical view of
% min. c'x
% s.t. x'x <= 1 (unit ball)
% in 2 dimensions

N = 10000;
n = 2; %number of dimensions
t = (2*rand(N,n)-1);
r = sqrt(t(:,1).^2 + t(:,2).^2);
x = t(r<=1,:); %kind of rejection sampling?
x = x';

% assume some non-zero c
c = [1 5];
c = c(:);

% compute c'x and plot!
v = c'*x;
figure, mesh(x(1,:),x(2,:),v);