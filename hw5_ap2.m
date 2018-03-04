% hw5 additional problem 2

% widths
w = ones(6,1);

% cap. loads
C_load_1 = 1.5;
C_load_2 = 1;
C_load_3 = 5;

% delays


% min, max widths
w_min = 0.1; w_max = 10;
w_x = linspace(w_min,w_max,10);

for ii = 1:length(w_x)
    C = w_x(ii)*w;
    R = 1./(w_x(ii)*w);

    T(1) = (C(3) + C_load_1)*(R(1) + R(2) + R(3)) + C(2)*(R(1) + R(2)) + ...
        (C(1) + C(4) + C(5) + C(6) + C_load_2 + C_load_3)*R(1);
    T(2) = (C(5) + C_load_2)*(R(1) + R(4) + R(5)) + C(4)*(R(1) + R(4)) + ...
        (C(6) + C_load_3)*(R(1) + R(4)) + (C(1) + C(2) + C(3) + C_load_1)*R(1);
    T(3) = (C(6) + C_load_3)*(R(1) + R(4) + R(6)) + C(4)*(R(1) + R(4)) + ...
        (C(1) + C(2) + C(3) + C_load_1)*R(1) + (C(5) + C_load_2)*(R(1) + R(4));
    
    T_max(ii) = max(T);
    A(ii) = sum(w_x(ii)*w);
end

figure, plot(A,T_max,'b-o','LineWidth',2);
xlabel('Area'); ylabel('Delay');

%% gp
mu = logspace(-3, 3, 10); 
w_opt = zeros(6,10);

for ii = 1:length(mu)
    mu_i = mu(ii);
    cvx_begin gp
        variable w(6)
        clear C R T;
        C = w;
        R = 1./w;
        T(1) = (C(3) + C_load_1)*(R(1) + R(2) + R(3)) + C(2)*(R(1) + R(2)) + ...
            (C(1) + C(4) + C(5) + C(6) + C_load_2 + C_load_3)*R(1);
        T(2) = (C(5) + C_load_2)*(R(1) + R(4) + R(5)) + C(4)*(R(1) + R(4)) + ...
            (C(6) + C_load_3)*(R(1) + R(4)) + (C(1) + C(2) + C(3) + C_load_1)*R(1);
        T(3) = (C(6) + C_load_3)*(R(1) + R(4) + R(6)) + C(4)*(R(1) + R(4)) + ...
            (C(1) + C(2) + C(3) + C_load_1)*R(1) + (C(5) + C_load_2)*(R(1) + R(4));
        
        minimize (sum(w) + mu_i * max(T))
        subject to
            w_min <= w <= w_max
    cvx_end
    if cvx_optval ~= Inf
        w_opt(:,ii) = w;
    else
        w_opt(:,ii) = -1*ones(6,1);
    end
end;


for ii = 1:10
    C = w_opt(:,ii);
    R = 1./w_opt(:,ii);

    T(1) = (C(3) + C_load_1)*(R(1) + R(2) + R(3)) + C(2)*(R(1) + R(2)) + ...
        (C(1) + C(4) + C(5) + C(6) + C_load_2 + C_load_3)*R(1);
    T(2) = (C(5) + C_load_2)*(R(1) + R(4) + R(5)) + C(4)*(R(1) + R(4)) + ...
        (C(6) + C_load_3)*(R(1) + R(4)) + (C(1) + C(2) + C(3) + C_load_1)*R(1);
    T(3) = (C(6) + C_load_3)*(R(1) + R(4) + R(6)) + C(4)*(R(1) + R(4)) + ...
        (C(1) + C(2) + C(3) + C_load_1)*R(1) + (C(5) + C_load_2)*(R(1) + R(4));
    
    T_max(ii) = max(T);
    A(ii) = sum(w_opt(:,ii));
end

hold on, plot(A,T_max,'r-x','LineWidth',2);
xlabel('Area'); ylabel('Delay'); title('Area-Delay tradeoff curve');
legend('Equal widths','CVX');