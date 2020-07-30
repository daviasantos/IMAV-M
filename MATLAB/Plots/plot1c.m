%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot1c
% Description: plot a scalar controlled variable against its commmand.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author:              Prof Dr Davi A Santos (ITA)
% Last modification:   January 26th, 2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function plot1c(t,r,l,r_,l_)
    
    figure; 
    
    plot(t,r,'LineWidth',2); grid; hold;
    plot(t,r_,'-','LineWidth',2); 
    leg1 = legend( l,l_ );
    set(leg1,'Interpreter','latex','FontSize',18);
    
end
