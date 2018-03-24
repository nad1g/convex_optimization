%% solution to additional problem A 3.16
clear all; close all; clc
thrusters_data

%% a) Optimal solution
cvx_begin
    variables u1(K-1) u2(K-1) p(2,K) v(2,K)
    minimize sum(u1 + u2)
    subject to
        % initial conditions
        p(:,1) == 0
        v(:,1) == 0
        % update equations
        f = [cos(theta1);sin(theta1)]*u1' + [cos(theta2);sin(theta2)]*u2' + repmat([0;-m*g],1,K-1);
        p(:,2:K) == p(:,1:K-1) + h*v(:,1:K-1);
        v(:,2:K) == (1 - alpha)*v(:,1:K-1) + (h/m) * f
        % waypoint constraints
        p(:,k1) == w1
        p(:,k2) == w2
        p(:,k3) == w3
        p(:,k4) == w4
        % constraints on p, u1 and u2
        norms(p,inf) <= pmax
        u1 >= 0
        u2 >= 0
cvx_end

% save the optimal value for use in part b
popt = cvx_optval;
figure, plot(p(1,:),p(2,:),'b-o');
hold on;
waypts = [w1 w2 w3 w4];
plot(waypts(1,:), waypts(2,:), 'rs', 'Linewidth',2);
xlabel('x'); ylabel('y'); title('Optimal trajectory');

%% b) explore epsilon-suboptimal points
clear p, v, u1, u2;
cvx_begin
    variables u1(K-1) u2(K-1) p(2,K) v(2,K)
    minimize sum(rand(K-1,1).*u1 + rand(K-1,1).*u2) + ...
             sum(rand(1,K).*p(1,:) + rand(1,K).*p(2,:)) + ...
             sum(rand(1,K).*v(1,:) + rand(1,K).*v(2,:))
    subject to
        % initial conditions
        p(:,1) == 0
        v(:,1) == 0
        % update equations
        f = [cos(theta1);sin(theta1)]*u1' + [cos(theta2);sin(theta2)]*u2' + repmat([0;-m*g],1,K-1);
        p(:,2:K) == p(:,1:K-1) + h*v(:,1:K-1)
        v(:,2:K) == (1 - alpha)*v(:,1:K-1) + (h/m) * f
        % waypoint constraints
        p(:,k1) == w1
        p(:,k2) == w2
        p(:,k3) == w3
        p(:,k4) == w4
        % constraints on p, u1 and u2
        norms(p,inf) <= pmax
        u1 >= 0
        u2 >= 0
        % constraint on 'f0'
        sum(u1+u2) <= popt *(1+0.01)
cvx_end
figure, plot(p(1,:),p(2,:),'b-x');
hold on
waypts = [w1 w2 w3 w4];
plot(waypts(1,:), waypts(2,:), 'rs', 'Linewidth',2);
xlabel('x'); ylabel('y'); title('sub-optimal trajectory');
