%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    IMAV-M - Flight dynamics and control simulator for MAVs 
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
% CControl
% Description: Flight control class. It implements the flight control laws,
% and control allocation.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Prof Dr Davi A Santos (ITA)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef CControl
    
    properties
    
        % Control parameters
        
        K1              % prop gain attitude control
        K2              % deriv gain attitude control
        K3              % prop gain position control
        K4              % deriv gain position control
        Kc              % prop gain velocity control 
        JB              % inertia matrix
        Jr              % rotor inertia 
        m               % mass
        g               % gravity
        nr              % number of rotors
        l               % arm length
        delta           % frontal half angle
        kf              % force coefficient
        kt              % torque coefficient
        k               % kt/kf
        Ginv            % inverse of control alloc matrix
        Tmin            % minimum torque
        Tmax            % maximum torque
        Fmin            % minimum force
        Fmax            % maximum force
        zetamin         % minimum virtual thrust
        zetamax         % maximum virtual thrust
        tau             % time const ref filter v_/wz_
        Ts              % sampling time
        alpha           % coeff ref filter v_/wz_
        
        % External variables
        
        r                % position 
        v                % velocity 
        D                % attitude matrix 
        W                % angular velocity 
        r_               % position command 
        v_               % velocity command
        vcheck           % output of ref filter of v_
        wzcheck          % output of ref filter of wz_
        p_               % heading command 
        wz_              % heading rate command 
        w_               % speed commands
        cax              % accel command in x (joystick)
        cay              % accel command in y (joystick)


        % Internal variable

        D_               % attitude matrix command 
        ea               % attitude error (3D representation)
        f_               % thrust commands
        TB_              % torque command in SB 
        FG_              % force command in SG 
        nG_              % normal direction command 
        F_               % force magnitude command 
        
        % Symbols
        
        e3               % (0,0,1)
        
   
    end
    
    methods
        
        %% Constructor
        
        function obj = CControl ( sControl )
            
            % Initialization
            
            obj.K1      = sControl.K1;
            obj.K2      = sControl.K2;
            obj.K3      = sControl.K3;
            obj.K4      = sControl.K4;
            obj.Kc      = sControl.Kc;
            obj.JB      = sControl.JB;
            obj.Jr      = sControl.Jr;
            obj.m       = sControl.m;
            obj.g       = sControl.g;
            obj.nr      = sControl.nr;
            obj.l       = sControl.l;
            obj.delta   = sControl.delta;
            obj.kf      = sControl.kf;
            obj.kt      = sControl.kt;
            obj.k       = sControl.k;
            obj.Tmin    = sControl.Tmin;
            obj.Tmax    = sControl.Tmax;
            obj.Fmin    = sControl.Fmin;
            obj.Fmax    = sControl.Fmax;
            obj.zetamin = sControl.zetamin;
            obj.zetamax = sControl.zetamax;
            obj.r_      = sControl.r_;
            obj.v_      = sControl.v_;
            obj.p_      = sControl.p_;
            obj.wz_     = sControl.wz_;
            obj.w_      = sControl.w_;
            obj.D_      = sControl.D_;
            obj.tau     = sControl.tau;
            obj.Ts      = sControl.Ts;
            
            obj.F_ = 0;
            obj.nG_ = [0;0;1];
            obj.TB_ = zeros(3,1);
            obj.FG_ = zeros(3,1);
            obj.vcheck = zeros(3,1);
            obj.wzcheck = 0;
            
            
            % Pre-computation
            
            obj.e3 = [0;0;1];
            obj.k = obj.kt/obj.kf;
            obj.Ginv = 0.25* [1 1/(obj.l*sind(obj.delta)) -1/(obj.l*cosd(obj.delta)) 1/obj.k;
                              1 -1/(obj.l*sind(obj.delta)) -1/(obj.l*cosd(obj.delta)) -1/obj.k;
                              1 -1/(obj.l*sind(obj.delta)) 1/(obj.l*cosd(obj.delta)) 1/obj.k;
                              1 1/(obj.l*sind(obj.delta)) 1/(obj.l*cosd(obj.delta)) -1/obj.k];
            
            obj.alpha = exp( -obj.Ts/obj.tau );
            
            
            
            
        end
        
        
        
        %% Position control
        
        function obj = PC( obj, mode )
            
            % Choose filtered ref in manual mode
            
            if ~mode 
                vaux_ = obj.vcheck;
            else
                vaux_ = obj.v_;
            end
            
            % Control law ifself
            
            obj.FG_ = obj.m*( obj.g*obj.e3 + obj.K3*(obj.r_-obj.r) + obj.K4*(vaux_-obj.v) );
            obj.FG_ = sat( obj.FG_, obj.Fmin, obj.Fmax );
           
            
            % Force magnitude and direction commands
            
            obj.F_  = norm( obj.FG_ );
            obj.nG_ = obj.FG_/obj.F_;
            
            
        end
        
        
        %% Velocity control
        
        function obj = VC( obj )
   
            % Control law ifself
            
            obj.FG_ = obj.m*( obj.g*obj.e3 + obj.Kc*(obj.vcheck-obj.v) );
            obj.FG_ = sat( obj.FG_, obj.Fmin, obj.Fmax );
           
            
            % Force magnitude and direction commands
            
            obj.F_  = norm( obj.FG_ );
            obj.nG_ = obj.FG_/obj.F_;
            
        end
        
        
        %% PB reference filter for manual mode 
        
        
        function obj = PBRefFilter( obj )
   
            obj.vcheck  = obj.alpha*obj.vcheck + (1-obj.alpha)*obj.v_;
            obj.wzcheck = obj.alpha*obj.wzcheck + (1-obj.alpha)*obj.wz_;
        
        end
        
        
        %% Attitude control
        
        function obj = AC( obj, mode )

            % Choose filtered ref in manual mode
            
            if ~mode 
                wzaux_ = obj.wzcheck;
            else
                wzaux_ = obj.wz_;
            end
            
            % Attitude error
            
            obj.ea = D2a( obj.D_*obj.D' );
            
            %  Control law itself

            obj.TB_ = cruz(obj.W)*( obj.JB*obj.W ) + ... 
                  obj.JB*( obj.K1*obj.ea + obj.K2*([0 0 wzaux_]'-obj.W) );
            obj.TB_ = sat( obj.TB_, obj.Tmin, obj.Tmax );    
            
   
        end
        
        
        %% Attitude command computation
        
        function obj = ATC( obj )
            
            % Convert from normal vector to phi-theta
            
            phi   = -atan2(obj.nG_(2),obj.nG_(3));
            theta =  asin(obj.nG_(1));
            
            psi   =  obj.p_;
            
            
               
            % Conversion to attitude matrix considering psi_      
            
            obj.D_ = a2D([phi theta psi]);

            
        end
        
        
        %% Attitude command computation for MANUAL state
        
        function obj = ATCman( obj )
            
            % Convert from normal vector to phi-theta
            
            phi1   = -atan(obj.nG_(2)/obj.nG_(3));
            theta1 =  asin(obj.nG_(1));
            
            % Convert joystick commands into attitude command
            
            phi2   = -obj.cay;
            theta2 =  obj.cax;
            psi   =  obj.p_;  
            
            % Conversion to attitude matrix considering psi_ 
            
            phi   = 0*phi1 + phi2;
            theta = 0*theta1 + theta2;
            
            obj.D_ = a2D([phi theta psi]);

            
        end
        
        
        %% Control allocation
        
        function obj = CA( obj )
                           
            % Thrust commands
            
            obj.f_ = obj.Ginv*[obj.F_;
                               obj.TB_];         

            obj.f_ = sat( obj.f_, obj.zetamin*ones(obj.nr,1), obj.zetamax*ones(obj.nr,1) );
            
            % Rotational speed commands
            
            for i=1:obj.nr
                
                obj.w_(i) = sqrt( obj.f_(i)/obj.kf );
            
            end
            
            
        end
        
        
        
    end
end

