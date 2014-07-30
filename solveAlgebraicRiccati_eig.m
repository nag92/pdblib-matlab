function X = solveAlgebraicRiccati_eig(A, G, Q)
%Solves the algebraic Riccati equation of the form A'X+XA'-XGX+Q=0, where X is symmetric with eigendecomposition.

n = size(A,1);
Z = [A -G; -Q -A'];

[V,D] = eig(Z);
U = [];
for j=1:2*n
	if real(D(j,j)) < 0 
		U = [U V(:,j)];
	end
end

X = U(n+1:end,1:n) / U(1:n,1:n);
X = real(X);