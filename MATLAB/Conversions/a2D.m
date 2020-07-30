%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Conversion from Euler angles 123 to attitude matrix
%  Author: Davi A Santos/ITA, 26/01/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function D = a2D(a)
               
D = [ cos(a(3))*cos(a(2)), cos(a(3))*sin(a(2))*sin(a(1))+sin(a(3))*cos(a(1)), ...
     -cos(a(3))*sin(a(2))*cos(a(1))+sin(a(3))*sin(a(1));
     -sin(a(3))*cos(a(2)), -sin(a(3))*sin(a(2))*sin(a(1))+cos(a(3))*cos(a(1)),...
      sin(a(3))*sin(a(2))*cos(a(1))+cos(a(3))*sin(a(1));
      sin(a(2)) , -cos(a(2))*sin(a(1)), cos(a(2))*cos(a(1))];
