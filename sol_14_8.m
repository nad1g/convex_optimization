spacecraft_landing_data
cvx_begin
variables p(3,K+1) v(3,K+1) f(3,K)
minimize sum(norms(f))
subject to
    norms(f) <= Fmax;
    v(:,1) == v0;
    p(:,1) == p0;
    v(:,2:K+1) == v(:,1:K) + (h/m) * f - repmat([0 0 1]',1,K)*h*g;
    p(:,2:K+1) == p(:,1:K) + (h/2) * (v(:,1:K) + v(:,2:K+1));
    p(3,2:K+1) >= alpha*norms(p(1:2,2:K+1));
    v(:,K+1) == 0;
    p(:,K+1) == 0;
cvx_end

%plot the glide cone (don't modify)
%-------------------------------------------------------
x = linspace(-40,55,30); y = linspace(0,55,30);
[X,Y] = meshgrid(x,y);
Z = alpha*sqrt(X.^2+Y.^2);
figure; colormap autumn; surf(X,Y,Z);
axis([-40,55,0,55,0,105]);
grid on; hold on;

%INSERT YOUR VARIABLES HERE:
%-------------------------------------------------------
plot3(p(1,:),p(2,:),p(3,:),'b','linewidth',1.5);
quiver3(p(1,1:K),p(2,1:K),p(3,1:K),...
        f(1,:),f(2,:),f(3,:),0.3,'k','linewidth',1.5);
