%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Conversion from quaternion to attitude matrix
%  Author: Davi A Santos/ITA, 26/01/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function D = q2D(q)
               
e = q(1:3);
n = q(4);

cross_e = cruz(e); 

D  = (n^2-e'*e)*eye(3) + 2*e*(e') - 2*n*cross_e;

         