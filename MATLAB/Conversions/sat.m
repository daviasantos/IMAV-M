%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Saturate a vector on a given set
%  Author: Prof Dr Davi A Santos (ITA)
%  Date: January 29th, 2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function xsat = sat(x,xmin,xmax)

    xsat = x;
  
    for i=1:size(x,1)
    
        if x(i) < xmin(i), xsat(i) = xmin(i); end
        if x(i) > xmax(i), xsat(i) = xmax(i); end

    end

end

