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
% CSensors
% Description: sensor platform class. It computes the sensor measurements
% from the system true states.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author:  Prof Dr Davi A Santos (ITA)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


classdef CSensors
    
    properties
         
        % measurements
        
        ya             
        yg
        ym
        yr
        yrp
        
        % biases (sensor states)
        
        ba
        bg
        bm
        
        % system states
        
        vp
        W
        r
        D
        
        % sensor platform parameters
        
        sa  
        sg  
        sm  
        sba 
        sbg 
        sbm 
        sr  
        g   
        mg  
        Ts  
        
        
        
        
    end
    
    
    methods
        
        %% Constructor
        
        function obj = CSensors( sSensors )
            
            % Initialization
            
            obj.ba  = sSensors.ba;
            obj.bg  = sSensors.bg;
            obj.bm  = sSensors.bm;
            obj.g   = sSensors.g;
            obj.mg  = sSensors.mg;   
            obj.sa  = sSensors.sa;
            obj.sg  = sSensors.sg;
            obj.sm  = sSensors.sm;
            obj.sba = sSensors.sba;
            obj.sbg = sSensors.sbg;
            obj.sbm = sSensors.sbm;
            obj.sr  = sSensors.sr;
            obj.Ts  = sSensors.Ts;
            obj.ya  = sSensors.ya;
            obj.yg  = sSensors.yg;
            obj.ym  = sSensors.ym;
            obj.yr  = sSensors.yr; 
            obj.yrp = sSensors.yrp;
            
            % Pre-computation
            
            
            
        end
        
        
        %% Simulate the accelerometer
        
        function obj = acc( obj )
        
            
            obj.ya = obj.D*(obj.vp + obj.g*[0;0;1]) + obj.ba + diag(obj.sa)*randn(3,1);
            
            obj.ba = obj.ba + obj.Ts*diag(obj.sba)*randn(3,1);
            
                
        end
        
        
        %% Simulate the rate-gyro
        
        function obj = gyro( obj )
                  
            obj.yg = obj.W + obj.bg + diag(obj.sg)*randn(3,1);
            
            obj.bg = obj.bg + obj.Ts*diag(obj.sbg)*randn(3,1);
            
                
        end
        
        
        %% Simulate the magnetometer
        
        function obj = mag( obj )
        
         
            obj.ym = obj.D*obj.mg + obj.bm + diag(obj.sm)*randn(3,1);
            
            obj.bm = obj.bm + obj.Ts*diag(obj.sbm)*randn(3,1);
            
                
        end
        
            
        %% Simulate the gps
        
        function obj = gps( obj )
        
            obj.yrp = obj.yr; 
            
            obj.yr = obj.r + diag(obj.sr)*randn(3,1);
            
            obj.yrp = (obj.yr - obj.yrp)/obj.Ts;
            
                
        end
        
        
    end
    
    
end

    