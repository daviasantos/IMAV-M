%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    IMAV-M Simulator - A flight dynamics and control simulator for MAVs 
%    Copyright (C) 2020  Aeronautics Institute of Technology
%
%    This program is free software: you can redistribute it and/or modify
%    it under the terms of the GNU General Public License as published by
%    the Free Software Foundation, either version 3 of the License, or
%    (at your option) any later version.
%
%    This program is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%    GNU General Public License for more details.
%
%    You should have received a copy of the GNU General Public License
%    along with this program. If not, see <https://www.gnu.org/licenses/>.
%
%    Also add information on how to contact you by electronic and paper mail.
%    To contact the author, please use the electronic address davists@ita.br or 
%    send a letter to
%    
%    Prof. Dr. Davi Antonio dos Santos
%    Divisao de Engenharia Mecanica
%    Instituto Tecnologico de Aeronautica
%    Praça Marechal Eduardo Gomes, 50, Vila das Acacias, 12228-900, Sao Jose dos Campos,
%    SP, Brasil.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% It is ppropriate for generating plots for scientific works.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:  Pure MATLAB simulation including: 
%
%           - plant and environment physics
%           - sensors
%           - guidance
%           - flight control laws
%           - attitude estimation (using mag, acc, gyro) and gps
%           - only auto waypoint state, for generating plots
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% Add to path

addpath('Plots');
addpath('Classes');
addpath('Conversions');


%% Clean and close

clc
clear
close all

tstart = tic;


%% Parameters

% Input parameters

load('Parameters.mat');



% MAV 

sMav.type = MAVType;
sMav.nr   = nr;
sMav.kf   = kf;
sMav.kt   = kt;
sMav.wmax = wmax;
sMav.km   = km;
sMav.Tm   = Tm;
sMav.l    = l;
sMav.delta= delta;
sMav.m    = m;
sMav.JB   = JB;
sMav.Jr   = Jr;
sMav.g    = g;
sMav.h    = Ts;
sMav.w    = zeros(nr,1);
sMav.r    = zeros(3,1);
sMav.v    = zeros(3,1);
sMav.vp   = zeros(3,1);
sMav.D    = eye(3);
sMav.W    = zeros(3,1);


oMav = CMav( sMav );




% Disturbance/Uncertainty 


sUncer.alpha_f = alpha_f;
sUncer.alpha_t = alpha_t;
sUncer.mg      = mg;
sUncer.tint    = -1;            % with -1, no interference at all
sUncer.mi      = 0;
sUncer.phi     = 90;

oUncer = CUncer( sUncer );



% Sensor platform 


sSensors.ba  = ba0;
sSensors.bg  = bg0;
sSensors.bm  = bm0;
sSensors.g   = g;
sSensors.mg  = mg;   
sSensors.sa  = sa;
sSensors.sg  = sg;
sSensors.sm  = sm;
sSensors.sba = sba;
sSensors.sbg = sbg;
sSensors.sbm = sbm;
sSensors.sr  = sr;
sSensors.Ts  = Ts;
sSensors.ya  = [0;0;g];
sSensors.yg  = zeros(3,1);
sSensors.ym  = mg;
sSensors.yr  = zeros(3,1); 
sSensors.yrp = zeros(3,1);


oSensors = CSensors( sSensors );


% Flight Control

sControl.K1      = K1;
sControl.K2      = K2;
sControl.K3      = K3;
sControl.K4      = K4;
sControl.Kc      = Kc;
sControl.JB      = JB;
sControl.Jr      = Jr;
sControl.m       = m;
sControl.g       = g;
sControl.nr      = nr;
sControl.l       = l;
sControl.delta   = delta;
sControl.kf      = kf;
sControl.kt      = kt;
sControl.k       = kt/kf;
sControl.Tmin    = Tmin;
sControl.Tmax    = Tmax;
sControl.Fmin    = Fmin;
sControl.Fmax    = Fmax;
sControl.zetamin = zetamin;
sControl.zetamax = zetamax;
sControl.r_      = zeros(3,1);
sControl.v_      = zeros(3,1);
sControl.p_      = 0;
sControl.wz_     = 0;
sControl.w_      = zeros(nr,1);
sControl.D_      = eye(3);
sControl.tau     = tau_ref_filter;
sControl.Ts      = Ts;

oControl = CControl( sControl );


% Trajectory planning/guidance 

sGuidance.nw    = nw;
sGuidance.wl    = wl;
sGuidance.Kpr   = Kpr;
sGuidance.Kpp   = Kpp;
sGuidance.Kdr   = Kdr;
sGuidance.Kdp   = Kdp;
sGuidance.rhor  = rhor;
sGuidance.rhop  = rhop;
sGuidance.dtl   = dtl;
sGuidance.Ts    = Ts;
sGuidance.dkl   = dtl/Ts;
sGuidance.l     = 1;
sGuidance.k     = 0;
sGuidance.r_    = zeros(3,1);
sGuidance.v_    = zeros(3,1);
sGuidance.p_    = 0;
sGuidance.wz_   = 0;
sGuidance.flag  = 0;


oGuidance = CGuidance( sGuidance );


% Navigation algorithm

sNavigation.tau = tau;
sNavigation.Ts  = Ts;
sNavigation.Ra  = Ra;
sNavigation.Rg  = Rg;
sNavigation.Rm  = Rm;
sNavigation.Rr  = Rr;
sNavigation.Qbg = Qbg;
sNavigation.mg  = mg;
sNavigation.x0  = x0;
sNavigation.P0  = P0;
            
oNavigation = CNavigation( sNavigation );



%% Initial interface

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp('IMAV-M 1.0: SIMULATION OF FLIGHT DYNAMICS AND CONTROL OF MAVs');
disp('Mode A: for generating plots.'); 
disp(' ');
disp('Description: Pure MATLAB simulation including:'); 
disp(' ');
disp('             - plant and environment physics');
disp('             - sensors');
disp('             - guidance');
disp('             - flight control laws');
disp('             - attitude estimation (using mag, acc, gyro) and gps');
disp('             - only auto waypoint state, for generating plots');
disp(' ');
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp('Author: Prof. Dr. Davi A. Santos (davists@ita.br)');
disp('Institution: Aeronautics Institute of Technology (ITA/Brazil)');
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp(' ');


input('Press ENTER to start...');
disp(' ');


%% Calibration


for k = 1:kfcalib 
    
    % Sensor measurements
    
    oSensors.vp = oMav.vp;
    oSensors.W  = oMav.W;
    oSensors.r  = oMav.r;
    oSensors.D  = oMav.D;
    
    oSensors = acc ( oSensors );
    oSensors = gyro( oSensors );
    oSensors = mag ( oSensors );
    oSensors = gps ( oSensors );
    
    
    % Navigation
    
    oNavigation.ya  = oSensors.ya;
    oNavigation.yg  = oSensors.yg;
    oNavigation.ym  = oSensors.ym;
    oNavigation.yr  = oSensors.yr;
    oNavigation.yrp = oSensors.yrp;
    
    
    oNavigation = NV( oNavigation );
    
    
end





%% Discrete-time loop

k=1;

while( 1 )
   
    
       
    
    %% Compute commands
    
        
    % input measurement

    oGuidance.r  = oNavigation.x(1:3); 
    oGuidance.v  = oNavigation.x(4:6); 
    aux          = D2a( q2D( oNavigation.x(10:13 )) );  
    oGuidance.p  = aux(3); 
    oGuidance.wz = oSensors.yg(3) - oNavigation.x(16); 


    % wayset verification

    oGuidance = wayset_verif( oGuidance );

    % wayset transition

    oGuidance = wayset_transit( oGuidance ); 

    % command generation

    oGuidance = plaw( oGuidance );


    % Commands to the flight controllers 

    oControl.r_    = oGuidance.r_;
    oControl.v_    = oGuidance.v_;
    oControl.p_    = oGuidance.p_; 
    oControl.wz_   = oGuidance.wz_;

  
    
    %% Flight control
    
    
    % Feedback variables 
    
    oControl.r     = oNavigation.x(1:3); 
    oControl.v     = oNavigation.x(4:6); 
    oControl.D     = q2D( oNavigation.x(10:13) );  
    oControl.W     = oSensors.yg - oNavigation.x(14:16);   
    
    
    % Position control law

    oControl = PC( oControl, 1 );
    
 
    % Attitude command computation
          
    oControl = ATC( oControl );
    
    
    % Attitude control law
     
    oControl = AC( oControl, 1 );
  
    
    % Control allocation algorithm
    
    oControl = CA( oControl );
    
   
    %% Environment and plant simulation
    
    
    % Disturbances 

    oUncer  = disturbances( oUncer );


    % Equations of motion

    oMav.Fd = oUncer.Fd;
    oMav.Td = oUncer.Td;
    oMav.w_ = oControl.w_;

    oMav = propeller( oMav );
    oMav = efforts  ( oMav );
    oMav = dynamics ( oMav );
    
    
    %% Magnetic interference simulation
    
    if oUncer.tint ~= -1 && k*Ts == oUncer.tint
      
        oUncer = interference( oUncer );
        
    end
    
    
    %% Sensor platform simulation
    
    oSensors.vp = oMav.vp;
    oSensors.W  = oMav.W;
    oSensors.r  = oMav.r;
    oSensors.D  = oMav.D;
    oSensors.bm = bm0 + oUncer.dm;
    
    oSensors = acc ( oSensors );
    oSensors = gyro( oSensors );
    oSensors = mag ( oSensors );
    oSensors = gps ( oSensors );
    

    %% Navigation algorithm
   
    
    oNavigation.ya  = oSensors.ya;
    oNavigation.yg  = oSensors.yg;
    oNavigation.ym  = oSensors.ym;
    oNavigation.yr  = oSensors.yr;
    oNavigation.yrp = oSensors.yrp;
    
    oNavigation = NV( oNavigation );
    
    
%% Monitoring 
    
    % Estimates
    
    red(:,k)  = oNavigation.x(1:3);
    ved(:,k)  = oNavigation.x(4:6);
    baed(:,k) = oNavigation.x(7:9);
    bged(:,k) = oNavigation.x(14:16);
    aed(:,k)  = 180/pi*D2a( q2D( oNavigation.x(10:13) ) );
    ead(:,k)  = 180/pi*oControl.ea;
    

    % True states
    
    rd(:,k)   = oMav.r; 
    vd(:,k)   = oMav.v;
    Wd(:,k)   = oMav.W;
    ad(:,k)   = 180/pi*D2a( oMav.D );
    
    % commands 
    
    rd_(:,k)  = oControl.r_;
    vd_(:,k)  = oControl.v_;
    ad_(:,k)  = 180/pi*D2a( oControl.D_ );
    wzd_(k)   = oControl.wz_;
    wd_(:,k)  = oControl.w_;
    FGd(:,k)  = oControl.FG_;
    TBd(:,k)  = oControl.TB_;  


%% Update time index (just for plotting)
    
    k = k + 1;
    if k == tf/Ts
        break;
    end

    
    
    
end 


%% Simulation time

tend = toc(tstart);
disp('Simulation finished! Duration:');
disp(' ');
disp(tend);



%% Plots

disp('Plotting...');
disp(' ');

t = 2*Ts:Ts:tf;

% position vs position command

plot1c(t,rd(1,:)','$r_1$',rd_(1,:)','$\bar{r}_1$');
plot1c(t,rd(2,:)','$r_2$',rd_(2,:)','$\bar{r}_2$');
plot1c(t,rd(3,:)','$r_3$',rd_(3,:)','$\bar{r}_3$');

% attitude vs attitude command

plot1c(t,ad(1,:)','$a_1$',ad_(1,:)','$\bar{a}_1$');
plot1c(t,ad(2,:)','$a_2$',ad_(2,:)','$\bar{a}_2$');
plot1c(t,ad(3,:)','$a_3$',ad_(3,:)','$\bar{a}_3$');


% force command 

plot3c(t,FGd','$F_1$','$F_2$','$F_3$');


% torque command

plot3c(t,TBd','$T_1$','$T_2$','$T_3$');


% 3D trajectory

plot3dc(rd,rd_,'Trajectory','Command');
 
   
% for evaluating/tuning the filters

plotEvalNavigation;
    




