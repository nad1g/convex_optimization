% illumination problem
clear all; close all; clc
illumdata
% n patches, m lamps.
[n,m] = size(A);

% a. equal lamp powers
g = 0.1:0.1:1;
p = repmat(g,10,1);
f_0p = max(abs(log(A*p)));
figure, plot(g,f_0p,'b-o');
xlabel('lamp power');
ylabel('objective');
% looks like 0.3 is best.
% choose p = 0.3
p = 0.3 * ones(m,1);
illu = A * p;
xx = floor(sqrt(n));
yy = floor(n/xx);
if (n ~= xx*yy)
    error('n is odd! cannot make illumination figures. quiting...')
end
illu = reshape(illu, yy, xx);
figure, surfc(1:1:xx, 1:1:yy, illu);
view(0,90); shading interp; colormap hot; colorbar;
title('illumination due to constant lamp power 0.3');

%b. least squares with saturation
b = ones(n,1);
p = A\b;
p(p>1) = 1;
p(p<0) = 0;
figure, stem(p,'k-x');
title('LS with saturation');
illu = A * p;
illu = reshape(illu, yy, xx);
figure, surfc(1:1:4, 1:1:5, illu);
view(0,90); shading interp; colormap hot; colorbar;
title('illumination due to LS w/saturation estimate');

%c. regularized least squares
rho = 0.1;
p = 1.1*ones(n,1);
while sum((p>1) + (p<0))
   %form the stacked A
   A_tilde = [A; sqrt(rho)*eye(size(A))];
   b_tilde = [ones(n,1); sqrt(rho)*0.5*ones(n,1)];
   p = A_tilde\b_tilde;
   fprintf(1, 'rho = %1.2f\n', rho);
   rho = rho + 0.1;
end
fprintf(1, 'rho selected = %1.2f\n', rho);
figure, stem(p, 'r-o');
title(['Regularized LS with rho = ', num2str(rho)]);
illu = A * p;
illu = reshape(illu, yy, xx);
figure, surfc(1:1:4, 1:1:5, illu);
view(0,90); shading interp; colormap hot; colorbar;
title('illumination due to reg. LS estimate');

%d. Chebyshev approximation
cvx_begin
variable p(m)
minimize norm(A*p - 1, inf)
subject to 
    -p <= 0
    p <= 1
cvx_end
figure, stem(p, 'g-s');
title('Chebyshev approximation');
illu = A * p;
illu = reshape(illu, yy, xx);
figure, surfc(1:1:4, 1:1:5, illu);
view(0,90); shading interp; colormap hot; colorbar;
title('illumination due to Chebyshev approx');

%e. piece-wise linear approximation of h(u) on points 0.5, 0.8, 1
clear p
cvx_begin
variable p(m)
u = A*p;
minimize (max(max([u, (2/0.5)-(1/0.5^2)*u, (2/.8) - (1/.8^2)*u, 2-u])))
subject to
    -p <= 0
    p <= 1
cvx_end
figure, stem(p, 'b-x');
title('PWL approximation');
illu = A * p;
illu = reshape(illu, yy, xx);
figure, surfc(1:1:4, 1:1:5, illu);
view(0,90); shading interp; colormap hot; colorbar;
title('illumination due to PWL approx')

% e. exact solution
clear p
cvx_begin
variable p(m)
minimize (max([A*p; inv_pos(A*p)]))
subject to
    -p <= 0;
    p <= 1;
cvx_end
figure, stem(p,'k-*');
title('Exact solution');
illu = A * p;
illu = reshape(illu, yy, xx);
figure, surfc(1:1:4, 1:1:5, illu);
view(0,90); shading interp; colormap hot; colorbar;
title('illumination (exact solution)')
