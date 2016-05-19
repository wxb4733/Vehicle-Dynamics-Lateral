%%  Nonlinear 4 DOF articulated vehicle model
%
%% Sintax
% |dx = _VehicleModel_.Model(~,estados)|
%
%% Arguments
% The following table describes the input arguments:
%
% <html> <table border=1 width="97%">
% <tr> <td width="30%"><tt>estados</tt></td> <td width="70%">Estados do modelo: [dPSI ALPHAT dPHI VEL PHI PSI XT YT]</td> </tr>
% </table> </html>
%
%% Description
% O �ngulo $\psi$ define a orienta��o do caminh�o-trator em rela��o ao referencial inercial. O estado $\phi$ � o �ngulo formado entre o caminh�o-trator e o semirreboque. O �ngulo $\alpha_T$ � o �ngulo de deriva do m�dulo dianteiro e � formado pelo vetor velocidade do centro de massa e a linha longitudinal do caminh�o-trator. Por fim, $v$ � o m�dulo do vetor velocidade do centro de massa do caminh�o-trator. Os pontos $T$ e $S$ s�o coincidentes com os centros de massa do caminh�o-trator e semirreboque, respectivamente. Os pontos F e R s�o coincidentes com os eixos dianteiro e traseiro do caminh�o-trator, respectivamente. M � o ponto que representa o eixo do semirreboque e A � o ponto de articula��o ente as duas unidades. As grandezas a, b e c da unidade motora s�o as dist�ncias entre os pontos F-T, T-R e R-A, respectivamente. Na unidade movida, d e e definem as dist�ncias entre os pontos A-S e S-M, respectivamente.
%
% <<illustrations/modeloArticulado.svg>>
%
% Este modelo � escrito na forma:
%
% $$ M(x) \dot{x} = f(x)$$
%
% Onde $x$ � o vetor de estados, $M(x)$ � a matriz de massa do sistema e $f(x)$ � uma fun��o vetorial n�o linear. Logo, � necess�ria uma fun��o que permita a integra��o do sistema com a matriz de massa escrita explicitamente. Uma op��o � utilizar a fun��o _ode45_. Details: <http://www.mathworks.com/help/matlab/ref/ode45.html?searchHighlight=%22mass%20matrix%22 ode45 (Mass matrix)>
%
%% Code
%

classdef VehicleArticulatedLinear3DOF < VehicleDynamicsLateral.VehicleArticulated
	methods
        % Constructor
        function self = VehicleArticulatedLinear3DOF(varargin)
            if nargin == 0
                % Entrada padr�o dos dados do ve�culo
                mF0 = 5237;         % Massa no eixo dianteiro do caminh�o-trator desacoplado [kg]
                mR0 = 2440;         % Massa no eixo traseiro do caminh�o-trator desacoplado [kg]
                mF = 6000;          % Massa no eixo dianteiro do caminh�o-trator (F) [kg]
                mR = 10000;         % Massa no eixo traseiro do caminh�o-trator (R) [kg]
                mM = 17000;         % Massa no eixo do semirreboque (M) [kg]
                IT = 46100;         % Momento de in�rcia do caminh�o-trator [kg*m2]
                IS = 452010;        % Momento de in�rcia do semirreboque [kg*m2]
                DELTA = 0*pi/180;          % Ester�amento do eixo dianteiro [rad]
                c = -0.310;         % Dist�ncia da articula��o ao eixo traseiro do caminh�o-trator (A-R) [m]
                lT = 3.550;         % Dist�ncia entre os eixos do caminh�o-trator [m]
                lS = 7.700;         % Dist�ncia entre a articula��o e o eixo do semirreboque [m]
                nF = 2;             % N�mero de tires no eixo dianteiro do caminh�o-trator
                nR = 4;             % N�mero de tires no eixo traseiro do caminh�o-trator
                nM = 8;             % N�mero de tires no eixo do semirreboque
                widthT = 2.6;     % width do caminh�o-trator [m]
                widthS = 2.550;   % width do semirreboque [m]
                muy = 0.3;          % Coeficiente de atrito de opera��o
                entradaVetor = [mF0 mR0 mF mR mM IT IS DELTA c lT lS nF nR nM widthT widthS muy];
                % Definindo os par�metros da classe
                self.params = self.convert(entradaVetor);
                self.tire = VehicleDynamicsLateral.TireLinear;
            else
                self.params = self.convert(varargin{1});
                self.tire = varargin{2};
            end
                self.distFT = self.params(20);
                self.distTR = self.params(21);
                self.distRA = self.params(9);
                self.distAS = self.params(22);
                self.distSM = self.params(23);
                self.width = self.params(15);
                self.widthSemi = self.params(16);
        end

        %% Model
        % Fun��o com as equa��es de estado do modelo
        function dx = Model(self,~,estados)
            % Dados do ve�culo
            mT = self.params(18);       % massa do veiculo [kg]
            mS = self.params(19);       % massa do veiculo [kg]
            IT = self.params(6);       % momento de inercia [kg]
            IS = self.params(7);       % momento de inercia [kg]
            a = self.params(20);        % distancia do eixo dianteiro ao centro de massa do caminh�o-trator [m]
            b = self.params(21);        % distancia do eixo traseiro ao centro de massa do caminh�o-trator [m]
            c = self.params(9);         % distancia da articula��o ao centro de massa do caminh�o-trator [m]
            d = self.params(22);        % distancia do eixo traseiro ao centro de massa do caminh�o-trator [m]
            e = self.params(23);        % distancia da articula��o ao centro de massa do caminh�o-trator [m]
            DELTA = self.params(8);     % Ester�amento [rad]
            nF = self.params(12);       % N�mero de tires no eixo dianteiro do caminh�o-trator
            nR = self.params(13);       % N�mero de tires no eixo traseiro do caminh�o-trator
            nM = self.params(14);       % N�mero de tires no eixo do semirreboque
            % g = 9.81;                   % Acelera��o da gravidade [m/s^2]
            % FzF = self.params(3)*g;     % Carga vertical no eixo dianteiro [N]
            % FzR = self.params(4)*g;     % Carga vertical no eixo traseiro [N]
            % FzM = self.params(5)*g;     % Carga vertical no eixo do semirreboque [N]
            % muy = self.params(17);      % Coeficiente de atrito de opera��o
            v0 = 20;
            % State variables
            X = estados(1,1);         % Not used
            Y = estados(2,1);         % Not used
            PSI     = estados(3,1);
            PHI     = estados(4,1);
            V       = estados(5,1);
            ALPHAT  = estados(6,1);
            dPSI    = estados(7,1);
            dPHI    = estados(8,1);

            % Slip angles
            ALPHAF = ALPHAT + a/v0*dPSI - DELTA;
            ALPHAR = ALPHAT - b/v0*dPSI;
            ALPHAM = ALPHAT + PHI - (dPSI*(b + c + d + e))/v0 + (dPHI*(d + e))/v0;

            % Longitudinal forces
            FxF = 0;
            FxR = 0;
            FxM = 0;

            % Lateral forces - Characteristic curve
            FyF = nF*self.tire.Characteristic(ALPHAF);%,FzF/nF,muy);
            FyR = nR*self.tire.Characteristic(ALPHAR);%,FzR/nR,muy);
            FyM = nM*self.tire.Characteristic(ALPHAM);%,FzM/nM,muy);

E = [ 1  0  0  0        0                        0                                               0                            0;...
      0  1  0  0        0                        0                                               0                            0;...
      0  0  1  0        0                        0                                               0                            0;...
      0  0  0  1        0                        0                                               0                            0;...
      0  0  0  0  (mS + mT)                      0                                               0                            0;...
      0  0  0  0        0             (v0*(mS + mT))                                 (-mS*(b + c + d))                         (d*mS);...
      0  0  0  0        0  (-mS*(d*v0 + v0*(b + c)))  (IS + IT + mS*((b + c)^2 + d*(2*b + 2*c) + d^2))  (- IS - mS*(d^2 + (b + c)*d));...
      0  0  0  0        0                  (d*mS*v0)                     (- IS - mS*(d^2 + (b + c)*d))                  (mS*d^2 + IS)];

A = [ 0  0   0  0  1   0                       0  0;...
      0  0  v0  0  0  v0                       0  0;...
      0  0   0  0  0   0                       1  0;...
      0  0   0  0  0   0                       0  1;...
      0  0   0  0  0   0                       0  0;...
      0  0   0  0  0   0           (-v0*(mS + mT))  0;...
      0  0   0  0  0   0  (mS*(d*v0 + v0*(b + c)))  0;...
      0  0   0  0  0   0                (-d*mS*v0)  0];

B = [ 0  0  0  0  0   0                0;...
      0  0  0  0  0   0                0;...
      0  0  0  0  0   0                0;...
      0  0  0  0  0   0                0;...
      0  1  1  1  0   0                0;...
      0  0  0  0  1   1                1;...
      0  0  0  0  a  -b  (- b - c - d - e);...
      0  0  0  0  0   0            (d + e)];


vetEst = [X ; Y ; PSI ; PHI ; V ; ALPHAT ; dPSI ; dPHI];
vetEnt = [DELTA ; FxF ; FxR ; FxM ; FyF ; FyR ; FyM];


            % Integrator output
            dx = E\A*vetEst + E\B*vetEnt;
        end

        %% Matriz de massa
        %

        function M = MassMatrix(self,~,estados)
            % Vehicle parameters
            mT = self.params(18);
            mS = self.params(19);
            IT = self.params(6);
            IS = self.params(7);
            % a = self.params(20);         % Not used
            b = self.params(21);
            c = self.params(9);
            d = self.params(22);

            % State variables
            % X = estados(1,1);         % Not used
            % Y = estados(2,1);         % Not used
            PSI = estados(3,1);
            PHI = estados(4,1);
            V = estados(5,1);
            ALPHAT = estados(6,1);
            % dPSI = estados(7,1);         % Not used
            % dPHI = estados(8,1);         % Not used

            % Mass matrix
            M55 = (mT + mS)*cos(PSI + ALPHAT);
            M56 = -(mT + mS)*V*sin(PSI + ALPHAT);
            M57 = mS*( (b+c)*sin(PSI) + d*sin(PSI - PHI) );
            M58 = -mS*d*sin(PSI - PHI);
            M65 = (mT + mS)*sin(PSI + ALPHAT);
            M66 = (mT + mS)*V*cos(PSI + ALPHAT);
            M67 = -mS*( (b+c)*cos(PSI) + d*cos(PSI - PHI) );
            M68 = mS*d*cos(PSI - PHI);
            M75 = -mS*( (b+c)*sin(ALPHAT) + d*sin(ALPHAT + PHI) );
            M76 = -mS*( (b+c)*V*cos(ALPHAT) + d*V*cos(ALPHAT + PHI) );
            M77 = mS*( (b+c)^2 + 2*(b+c)*d*cos(PHI) + d^2 ) + IT + IS;
            M78 = -( mS*( (b+c)*d*cos(PHI) + d^2 ) + IS);
            M85 = mS*d*sin(ALPHAT + PHI);
            M86 = mS*d*V*cos(ALPHAT + PHI);
            M87 = - (mS*(d^2 + (b+c)*d*cos(PHI)) + IS);
            M88 = mS*d^2 + IS;

            M = [   1 0 0 0  0   0   0   0 ;...
                    0 1 0 0  0   0   0   0 ;...
                    0 0 1 0  0   0   0   0 ;...
                    0 0 0 1  0   0   0   0 ;...
                    0 0 0 0 M55 M56 M57 M58 ;...
                    0 0 0 0 M65 M66 M67 M68 ;...
                    0 0 0 0 M75 M76 M77 M78 ;...
                    0 0 0 0 M85 M86 M87 M88 ];

        end



    end

    methods (Static)
        %% convert
        % A fun��o convert adiciona no vetor de entrada ([mF0 mR0 mF mR mM IT IS DELTA c lT lS nF nR nM widthT widthS muy]) os par�metros restantes do modelo de ve�culo ([mT mS a b d e]).
        function parametros = convert(entrada)
            mF0 = entrada(1);       % Massa no eixo dianteiro do caminh�o-trator desacoplado [kg]
            mR0 = entrada(2);       % Massa no eixo traseiro do caminh�o-trator desacoplado [kg]
            mF = entrada(3);        % Massa no eixo dianteiro do caminh�o-trator (F) [kg]
            mR = entrada(4);        % Massa no eixo traseiro do caminh�o-trator (R) [kg]
            mM = entrada(5);        % Massa no eixo do semirreboque (M) [kg]
            lT = entrada(10);       % Dist�ncia entre os eixos do caminh�o-trator [m]
            lS = entrada(11);       % Dist�ncia entre a articula��o e o eixo do semirreboque [m]
            % Convers�o dos dados para os par�metros usados na equa��o de movimento
            g = 9.81;               % Acelera��o da gravidade [m/s^2]
            mT = mF0 + mR0;         % massa do caminh�o-trator [kg]
            a = mR0/mT*lT;          % Dist�ncia do eixo dianteiro ao CG do caminh�o-trator (F-T) [m]
            b = lT - a;             % Dist�ncia do eixo traseiro ao CG do caminh�o-trator (R-T) [m]
            A = mF*g + mR*g - mT*g; % For�a vertical na articula��o [N]
            mS = (A + mM*g)/g;      % massa do semirreboque [kg]
            d = (lS*mM)/mS;         % Dist�ncia da articula��o ao CG do semirreboque (A-S) [m]
            e = lS - d;             % Dist�ncia do eixo traseiro ao CG do semirreboque (M-S) [m]
            % Sa�da
            parametros = [entrada mT mS a b d e];
        end
    end

    %% Properties
    %

    properties
        params
        tire
        distFT
        distTR
        distRA
        distAS
        distSM
        width     % width do caminh�o-trator
        widthSemi % width do semirreboque
    end
end


%% See Also
%
% <index.html Index> | <VehicleSimpleNonlinear3DOF.html VehicleSimpleNonlinear3DOF>
%
