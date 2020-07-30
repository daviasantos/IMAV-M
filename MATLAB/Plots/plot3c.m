%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot3c
% Description: plot the three components of a vector sequence
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author:              Prof Dr Davi A Santos (ITA)
% Last modification:   January 26th, 2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function plot3c(t,F,l1,l2,l3)
    
    figure; 
    
    plot(t,F,'LineWidth',2); grid; hold;
    leg1 = legend(l1,l2,l3);
    set(leg1,'Interpreter','latex','FontSize',18);
    

    
end