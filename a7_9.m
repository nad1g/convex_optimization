% A7.9

P1 = [1 0 0 0; 0 1 0 0; 0 0 1 0];
P2 = [1 0 0 0; 0 0 1 0; 0 -1 0 10];
P3 = [1 1 1 -10; -1 1 1 0; -1 -1 1 10];
P4 = [0 1 1 0; 0 -1 1 0; -1 0 0 10];

y1 = [0.98; .93]; y2 = [1.01; 1.01]; y3 = [0.95; 1.05]; y4 = [2.04; 0];

tol = 1e-5;
l = 0; u = 1;
x_opt = -1*ones(3,1);

while (u-l > tol)
   t = (l+u)/2;
   disp(['trying t = ', num2str(t)]);
 
   cvx_begin quiet
    variable x(3)
    
    z1 = P1*[x;1]; %z1(1:2) = Ax + b, z1(3) = c'x + d
    norm(z1(1:2) - y1*z1(3)) <= t*z1(3);
    
    z2 = P2*[x;1];
    norm(z2(1:2) - y2*z2(3)) <= t*z2(3);
    
    z3 = P3*[x;1];
    norm(z3(1:2) - y3*z3(3)) <= t*z3(3);
    
    z4 = P4*[x;1];
    norm(z4(1:2) - y4*z4(3)) <= t*z4(3);
   cvx_end
   
   if (cvx_optval ~= Inf) % feasible 'x'
       u = t;
       x_opt = x;
   else
       l = t;
   end
   
 end

disp(x_opt);
