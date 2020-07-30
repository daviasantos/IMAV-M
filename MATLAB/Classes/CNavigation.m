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
% CNavigation
% Description: navigation class. It implements an attitude EKF from vector 
% measurements token from acc and mag. Gyro measurements are used for state
% prediction. It uses gps to obtain position measurements. Velocity is 
% estimated by numerical derivative of position and LP filter.
% => State vector: (r,v,q,ba,bg).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Prof Dr Davi A Santos (ITA)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef CNavigation
    
    properties
        
        % Symbols
        
        O33
        O34
        O43
        I3
        I16
        e3

        % Parameters
        
        Ra                  % cov acc
        Rg                  % cov gyro
        Rm                  % cov mag
        Rr                  % cov gps
        Rrp                 % cov gps deriv
        Qba                 % cov wiener process bias acc
        Qbg                 % cov wiener process bias gyro
        Qbm                 % cov wiener process bias mag
        R                   % cov meas 
        Q                   % cov state
        mg                  % local mag den sity         
        Ts                  % sampling time
        tau                 % time const LP filter of gps deriv
        
        
        % Input variables
        
        ya                  % accelerometer measurement
        yg                  % gyro measurement
        ym                  % magnetometer measurement 
        yr                  % gps position measurement
        yrp                 % gps velocity measurement
        

        % state variables

        x                   % State estimate 
        P                   % State estimate covariance 
        yt                  % innovation 
        Py                  % innov cov
     
   
    end
    
    
    
    methods
    
        %% Constructor
        
        function obj = CNavigation( sNavigation )
            
            % Initialize internal symbols
            
            obj.O33 = zeros(3,3);
            obj.O34 = zeros(3,4);
            obj.O43 = zeros(4,3);
            obj.I3  = eye(3);
            obj.I16 = eye(16);
            obj.e3  = [0;0;1];
            
            % Parameters
            
            obj.tau = sNavigation.tau;
            obj.Ts  = sNavigation.Ts;
            obj.Ra  = sNavigation.Ra;
            obj.Rg  = sNavigation.Rg;
            obj.Rm  = sNavigation.Rm;
            obj.Rr  = sNavigation.Rr;
            obj.Qbg = sNavigation.Qbg;
            
            obj.Rrp = 2/(obj.Ts^2)*obj.Rr;
            
            obj.Q   = [obj.Rg,obj.O33;
                       obj.O33,obj.Qbg];
            
            obj.R   =  [obj.Ra,obj.O33;
                        obj.O33,obj.Rm];
            
            obj.mg =  sNavigation.mg/norm(sNavigation.mg);
                    
                    
            % Initial conditions 
            
            obj.yr   = zeros(3,1);
            obj.x    = sNavigation.x0;
            obj.P    = sNavigation.P0;
            
            
        
        end
        
        
        
        %% Navigation algorithm
        
        function obj = NV ( obj )
        
            
            % Initilize variables
      
            x_AD  = obj.x(10:16);
            P_AD  = obj.P(10:16,10:16);
            y_AD  = [obj.ya/norm(obj.ya);obj.ym/norm(obj.ym)];
            u_AD  = obj.yg;
            
            % Prediction
            
            Faux = F_AD( x_AD,u_AD );
            Gaux = G_AD( x_AD );
            k1x  = obj.Ts*f_AD( x_AD,u_AD );
            k1P  = obj.Ts*( Faux*P_AD + P_AD*Faux' + Gaux*obj.Q*Gaux' );
            

            Faux = F_AD( x_AD+k1x/2,u_AD );
            Gaux = G_AD( x_AD+k1x/2 );
            k2x  = obj.Ts*f_AD( x_AD+k1x/2,u_AD );
            k2P  = obj.Ts*( Faux*(P_AD+k1P/2) + (P_AD+k1P/2)*Faux' + Gaux*obj.Q*Gaux');
          

            Faux = F_AD( x_AD+k2x/2,u_AD );
            Gaux = G_AD( x_AD+k2x/2 );
            k3x  = obj.Ts*f_AD( x_AD+k2x/2,u_AD );
            k3P  = obj.Ts*( Faux*(P_AD+k2P/2) + (P_AD+k2P/2)*Faux' + Gaux*obj.Q*Gaux');
           

            Faux = F_AD( x_AD+k3x,u_AD );
            Gaux = G_AD( x_AD+k3x );
            k4x  = obj.Ts*f_AD( x_AD+k3x,u_AD );
            k4P  = obj.Ts*( Faux*(P_AD+k3P) + (P_AD+k3P)*Faux' + Gaux*obj.Q*Gaux');
           

            x_AD  = x_AD + k1x/6 + k2x/3 + k3x/3 + k4x/6;
            P_AD  = P_AD + k1P/6 + k2P/3 + k3P/3 + k4P/6;


            % Compute ye, Py, Pxy

            Hk1    = H_AD( x_AD );
            ye_AD  = h_AD( x_AD );
            Py_AD  = Hk1*P_AD*Hk1' + obj.R;
            Pxy_AD = P_AD*Hk1'; 
            
            
            % Update

            K_AD = Pxy_AD/Py_AD;

            yt_AD = y_AD - ye_AD;
            x_AD  = x_AD + K_AD*yt_AD;
            P_AD  = P_AD - K_AD*Pxy_AD';

            % quaternion normalization
            
            x_AD(1:4) = x_AD(1:4)/norm( x_AD(1:4) );
            
  
            % Localization

            obj.x(1:3) = obj.yr;
            
            yrpf = obj.x(4:6);
            yrpf = ( 1-obj.Ts/obj.tau )*yrpf + obj.Ts/obj.tau*obj.yrp;
            obj.x(4:6) = yrpf;

            obj.x(7:9)     = zeros(3,1);
            obj.P(1:9,1:9) = blkdiag(obj.Rr,obj.Rrp,obj.O33);
            
            yt_LO = zeros(9,1);
            Py_LO = zeros(9,9);

        
            % full state estimate
            
            obj.x(10:16)       = x_AD;
            obj.P(10:16,10:16) = P_AD;

            
            % Innovation sequence

            obj.yt = [yt_LO;yt_AD];
            obj.Py = blkdiag(Py_LO,Py_AD);
            
        
        
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % CHILD FUNCTIONS
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %% State function

            function xp = f_AD( xe,u )

                % variables 

                w  = u;
                q  = xe(1:4);
                bg = xe(5:7);

                % x dot

                xp = [1/2*WW(w-bg)*q;
                      zeros(3,1)];


            end


            %% State Jacobian

            function Fe = F_AD( xe,u )

                % variables and parameters

                w    = u;
                q    = xe(1:4);
                bg   = xe(5:7);

                % Jacobian matrix

                Fe = [1/2*WW(w-bg),-1/2*QQ(q);
                      obj.O34, obj.O33];

            end


            %% State-dependent noise matrix

            function Ga = G_AD( xe )

                % variables and parameters

                q   = xe(1:4);


                % G matrix

                Ga = [-1/2*QQ(q),obj.O43;
                       obj.O33,obj.I3];


            end


            %% Measurement function

            function ye = h_AD( xe )

                % variables and parameters

                q  = xe(1:4);


                % h function

                ye = [q2D(q)*obj.e3;
                      q2D(q)*obj.mg];


            end



            %% Measurement Jacobian

            function He = H_AD( xe )

                % variables

                q = xe(1:4);


                % Partial matrices

                H11 = 2*[q(3) -q(4) q(1) -q(2);
                         q(4) q(3) q(2) q(1);
                        -q(1) -q(2) q(3) q(4)];
                H12 = zeros(3,3);

                dDdq1 = 2*[q(1) q(2) q(3);
                           q(2) -q(1) q(4);
                           q(3) -q(4) -q(1)];
                dDdq2 = 2*[-q(2) q(1) -q(4);
                            q(1) q(2) q(3);
                            q(4) q(3) -q(2)];
                dDdq3 = 2*[-q(3) q(4) q(1);
                           -q(4) -q(3) q(2);
                            q(1) q(2) q(3)];
                dDdq4 = 2*[ q(4) q(3) -q(2);
                           -q(3) q(4) q(1);
                            q(2) -q(1) q(4)];

                H21 = [dDdq1*obj.mg,dDdq2*obj.mg,dDdq3*obj.mg,dDdq4*obj.mg];
                H22 = zeros(3,3);

                % Jacobian matrix

                He  = [H11, H12;
                       H21, H22];


            end



            %% W script matrix

            function Wa = WW( w )

                Wa = [-cruz(w),w;
                      -w',0];

            end


            %% Q script matrix

            function Qa = QQ(q)

                % variables 

                n = q(4);
                e = q(1:3);
                ex = cruz(e);

                % matrix Q(q)

                Qa = [ex+n*eye(3);
                      -e'];

            end

        
        
        
        end
        
        
        
        
        
        
            
    end
    
end

