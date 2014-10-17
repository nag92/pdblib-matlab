function X = solveAlgebraicRiccati_Schur(A, G, Q)
%Solves the algebraic Riccati equation of the form A'X+XA'-XGX+Q=0, where X is symmetric.
%
%@article{Paige81,
%  author = "Paige, C. and Van Loan, C.",
%  title = "A {S}chur decomposition for {H}amiltonian matrices",
%  journal = "Linear Algebra and its Applications",
%  volume = "41",
%  pages = "11--32",
%  year = "1981",
%}

n = size(A,1);
Z = [A -G; -Q -A']; %See Eq. (5.2.3) in doc/TechnicalReport.pdf

[U0, S0] = schur(Z); %See Eq. (5.2.6) in doc/TechnicalReport.pdf
U = ordschur(U0, S0, 'lhp'); %Not yet available in GNU Octave (See Eq. (5.2.6) in doc/TechnicalReport.pdf)
X = U(n+1:end,1:n) / U(1:n,1:n); %See Eq. (5.2.7) in doc/TechnicalReport.pdf