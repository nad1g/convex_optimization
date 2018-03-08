% Solution to T4.41 (a)

% A in S^n is copositive if x'Ax >= 0 for all x >= 0
% A can be expressed as A = B + C where B is pos. sem
% and C is elementwise non negative (C(i,j) >= 0)
% Finding B,C is an SDP feasibility problem.

N = 2;
A = rand(N,N);
A = (A + A') + rand(1)*eye(N); % ensure positive definiteness

cvx_begin sdp
    variable B(N,N) semidefinite
    variable C(N,N)
    A == B + C
    B >= 0
    C(:) >= 0 % elementwise non-negative.
cvx_end