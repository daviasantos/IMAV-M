%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot3dc
% Description: plot the vector and command in 3D
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author:              Prof Dr Davi A Santos (ITA)
% Last modification:   January 26th, 2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function plot3dc(r,r_,l1,l2)
    
    figure; 
    plot3(r(1,:),r(2,:),r(3,:),'color','b','LineWidth',2); hold;
    plot3(r_(1,:),r_(2,:),r_(3,:),':r','LineWidth',2);
    box; grid;
    
    leg1 = legend(l1,l2);
    set(leg1,'Interpreter','latex','FontSize',18);
    

    
end