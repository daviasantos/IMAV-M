%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Conversion from attitude matrix to Euler angles 123
%  Author: Davi A Santos/ITA, 26/01/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function a = D2a(D)
        
     phi = -atan2(D(3,2),D(3,3));
            
     D31 = D(3,1);
     if abs(D31)>1, D31 = sign(D31); end
            
     theta =  asin(D31);
     psi   = -atan2(D(2,1),D(1,1));
            
     a = [phi,theta,psi]';

end
    