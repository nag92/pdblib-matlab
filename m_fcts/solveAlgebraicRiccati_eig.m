function X = solveAlgebraicRiccati_eig(A, G, Q)
%Solves the algebraic Riccati equation of the form A'X+XA'-XGX+Q=0, where X is symmetric with eigendecomposition.
%Danilo Bruno, 2014

n = size(A,1);
Z = [A -G; -Q -A']; %See Eq. (5.2.3) in doc/TechnicalReport.pdf

[V,D] = eig(Z); %See Eq. (5.2.4) in doc/TechnicalReport.pdf
U = [];
for j=1:2*n
	if real(D(j,j)) < 0
		U = [U V(:,j)];
	end
end

X = U(n+1:end,1:n) / U(1:n,1:n); %See Eq. (5.2.5) in doc/TechnicalReport.pdf
X = real(X);