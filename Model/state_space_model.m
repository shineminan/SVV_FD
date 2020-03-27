% Citation 550 - Linear simulation

ft2m = 0.3048;
lbs2kg = 0.4536;
kts2mps = 0.5144;
in2m = 0.0254;

% 1: Short period
% 2: Phugoid
% 3: Dutch roll
% 5: Aperiodic roll
% 6: Spiral
motion = 6;

% % For test
% whichone = 'test';
% m_fuel = 4100*lbs2kg;           % Fuel mass [lbs]->[kg]
% % aerodynamic properties
% e      = 0.4954;                % Oswald factor [ ]
% CD0    = 0.0230;                % Zero lift drag coefficient [ ]
% CLa    = 0.0771/deg2rad(1);     % Slope of CL-alpha curve [ ]
% % Longitudinal stability
% Cma    = -0.0114/deg2rad(1);    % longitudinal stabilty [ ]
% Cmde   = -0.0252/deg2rad(1);    % elevator effectiveness [ ]

% For reference
whichone = 'reference';
m_fuel = 4050*lbs2kg;           % Fuel mass [lbs]->[kg]
% aerodynamic properties
e      = 0.6522;                % Oswald factor [ ]
CD0    = 0.0236;                % Zero lift drag coefficient [ ]
CLa    = 0.0787/deg2rad(1);     % Slope of CL-alpha curve [ ]
% Longitudinal stability
Cma    = -0.0119/deg2rad(1);    % longitudinal stabilty [ ]
Cmde   = -0.0255/deg2rad(1);    % elevator effectiveness [ ]

% Get timestamps for eigenmotion from measurement data
load(strcat(whichone, '_data.mat'));
load(strcat(whichone, '_eigenmotion.mat'));
dt = 0.1;
i_em = int32((eigenmotion.data(motion,:)-flightdata.time.data(1))/dt+1);
is = i_em(1):i_em(2);
Tfinal = eigenmotion.data(motion,2)-eigenmotion.data(motion,1);

% Inputs de da dr
de = deg2rad(flightdata.delta_e.data(is));  % Deflection of elevator [rad]
da = deg2rad(flightdata.delta_a.data(is));  % Deflection of aileron [rad]
dr = deg2rad(flightdata.delta_r.data(is));  % Deflection of rudder [rad]

% Mass of fuel used till the beginning of this eigenmotion
ful = flightdata.lh_engine_FU.data(i_em(1))*lbs2kg;     % Fuel used of left engine [lbs]->[kg]
fur = flightdata.rh_engine_FU.data(i_em(1))*lbs2kg;     % Fuel used of right engine [lbs]->[kg]
% Aircraft mass at the beginning of the eigenmotion
load(strcat(whichone,'_payload.mat'));
m_PL = sum(PL.data(:,1));                   % Payload mass [kg]
m_OE = 9165*lbs2kg;                         % Operational empty mass [lbs]->[kg]
m = m_OE + m_PL + m_fuel - (ful+fur);       % Total a/c mass [kg]

% % Stationary flight condition
hp0 = flightdata.Dadc1_alt.data(i_em(1))*ft2m;              % pressure altitude in the stationary flight condition [m]
V0 = flightdata.Dadc1_tas.data(i_em(1))*kts2mps;            % true airspeed in the stationary flight condition [m/sec]
alpha0 = deg2rad(flightdata.vane_AOA.data(i_em(1)));        % angle of attack in the stationary flight condition [rad]
th0 = deg2rad(flightdata.Ahrs1_Pitch.data(i_em(1)));        % pitch angle in the stationary flight condition [rad]
phi0 = deg2rad(flightdata.Ahrs1_Pitch.data(i_em(1)));       % pitch angle [rad]

% Angular velocity
q_meas = deg2rad(flightdata.Ahrs1_bPitchRate.data(is));     % pitch rate [rad/s]
r_meas = deg2rad(flightdata.Ahrs1_bYawRate.data(is));       % yaw rate [rad/s]
p_meas = deg2rad(flightdata.Ahrs1_bRollRate.data(is));      % roll rate [rad/s]


% Aircraft geometry
S      = 30.00;	          % wing area [m^2]
Sh     = 0.2*S;           % stabiliser area [m^2]
Sh_S   = Sh/S;	          % [ ]
lh     = 0.71*5.968;      % tail length [m]
c      = 2.0569;          % mean aerodynamic cord [m]
lh_c   = lh/c;	          % [ ]
b      = 15.911;	      % wing span [m]
bh     = 5.791;	          % stabilser span [m]
A      = b^2/S;           % wing aspect ratio [ ]
Ah     = bh^2/Sh;         % stabilser aspect ratio [ ]
Vh_V   = 1;		          % [ ]
ih     = -2*pi/180;       % stabiliser angle of incidence [rad]

% Constant values concerning atmosphere and gravity
rho0   = 1.2250;          % air density at sea level [kg/m^3] 
lambda = -0.0065;         % temperature gradient in ISA [K/m]
Temp0  = 288.15;          % temperature at sea level in ISA [K]
R      = 287.05;          % specific gas constant [m^2/sec^2K]
g      = 9.81;            % [m/sec^2] (gravity constant)

rho    = rho0*(1+lambda*hp0/Temp0)^(-g/(lambda*R)+1);   % [kg/m^3]  (air density)
W      = m*g;                                           % [N]       (aircraft weight)

% Constant values concerning aircraft inertia
muc    = m/(rho*S*c);
mub    = m/(rho*S*b);
KX2    = 0.019;
KZ2    = 0.042;
KXZ    = 0.002;
KY2    = 1.25*1.114;

% Aerodynamic constants
Cmac   = 0;                     % Moment coefficient about the aerodynamic centre [ ]
CNwa   = CLa;   		        % Wing normal force slope [ ]
CNha   = 2*pi*Ah/(Ah+2);        % Stabiliser normal force slope [ ]
depsda = 4/(A+2);               % Downwash gradient [ ]

% Lift and drag coefficient
CL = 2*W/(rho*V0^2*S);               % Lift coefficient [ ]
CD = CD0 + (CLa*alpha0)^2/(pi*A*e);  % Drag coefficient [ ]

% Stabiblity derivatives
CX0    = W*sin(th0)/(0.5*rho*V0^2*S);
CXu    = -0.09500;
CXa    = -0.47966;
CXadot = +0.08330;
CXq    = -0.28170;
CXde   = -0.03728;

CZ0    = -W*cos(th0)/(0.5*rho*V0^2*S);
CZu    = -0.37616;
CZa    = -5.74340;
CZadot = -0.00350;
CZq    = -5.66290;
CZde   = -0.69612;

Cmu    = +0.06990;
Cmadot = +0.17800;
Cmq    = -8.79415;

CYb    = -0.7500;
CYbdot =  0     ;
CYp    = -0.0304;
CYr    = +0.8495;
CYda   = -0.0400;
CYdr   = +0.2300;

Clb    = -0.10260;
Clp    = -0.71085;
Clr    = +0.23760;
Clda   = -0.23088;
Cldr   = +0.03440;

Cnb    =  +0.1348;
Cnbdot =   0     ;
Cnp    =  -0.0602;
Cnr    =  -0.2061;
Cnda   =  -0.0120;
Cndr   =  -0.0939;


% Symmetric state-space
% States [u_hat, alpha, theta, qc/V]
% u_hat = (Vt-Vt0)/Vt0
% alpha = alpha_body - alpha_0
% theta = theta_body - theta_0
Ps = [-2*muc*c/V0   0                       0       0;
      0             (CZadot-2*muc)*c/V0     0       0;
      0             0                       -c/V0   0;
      0             Cmadot*c/V0             0       -2*muc*KY2*c/V0];
Qs = [-CXu      -CXa    -CZ0    -CXq;
      -CZu      -CZa    CX0     -(CZq+2*muc);
      0         0       0       -1;
      -Cmu      -Cma    0       -Cmq];
Rs = [-CXde; -CZde; 0; -Cmde];

As = Ps\Qs;
Bs = Ps\Rs;
Cs = eye(4);
Ds = 0;
syss = ss(As,Bs,Cs,Ds,'StateName',{'u_hat' 'alpha' 'theta' 'qc/V'},'InputName','delta_e','OutputName',{'(V/V_0)/V','\alpha','\theta','qc/V'});

% Asymmetric state-space
% States [beta, phi, p*b/(2*V), r*b/(2*V)]
% u_hat = (Vt-Vt0)/Vt0
% p*b/(2*V) = p_meas*b/(2*Vt0)
% r*b/(2*V) = r_meas*b/(2*Vt0)
Pa = [(CYbdot-2*mub)*b/V0   0           0                   0;
      0                     -b/(2*V0)   0                   0;
      0                     0           -4*mub*KX2*b/V0     4*mub*KXZ*b/V0;
      Cnbdot*b/V0           0           4*mub*KXZ*b/V0      -4*mub*KZ2*b/V0];
Qa = [-CYb  -CL     -CYp    -(CYr-4*mub);
      0     0       -1      0;
      -Clb  0       -Clp    -Clr;
      -Cnb  0       -Cnp    -Cnr];
Ra = [-CYda  -CYdr;
     0      0;
     -Clda  -Cldr;
     -Cnda  -Cndr];
Aa = Pa\Qa;
Ba = Pa\Ra;
Ca = eye(4);
Da = 0;
sysa = ss(Aa,Ba,Ca,Da,'StateName',{'beta' 'phi' 'pb/(2V)' 'rb/(2V)'},'InputName',{'delta_a' 'delta_r'});


duration = 0:dt:Tfinal;
if motion==1 || motion==2
    out = lsim(syss,de,duration);
    
    TAS_sim = out(:,1)*V0+V0;
    alpha_sim = rad2deg(out(:,2) + alpha0);
    theta_sim = rad2deg(out(:,3) + th0);
    qcv_sim = out(:,4);
    
    subplot(4,1,1)
    plot(duration, TAS_sim, duration,flightdata.Dadc1_tas.data(is)*kts2mps,'LineWidth',5)
    legend('Simulation', 'Flight test','FontSize',14);
    xlabel('Time [s]', 'FontSize', 16);
    ylabel('V_t [m/s]', 'FontSize', 16);
    
    subplot(4,1,2)
    plot(duration,alpha_sim, duration,flightdata.vane_AOA.data(is),'LineWidth',5)
    legend('Simulation', 'Flight test','FontSize',14);
    xlabel('Time [s]', 'FontSize', 16);
    ylabel('\alpha [deg]', 'FontSize', 16);
    
    subplot(4,1,3)
    plot(duration,theta_sim, duration,flightdata.Ahrs1_Pitch.data(is),'LineWidth',5)
    legend('Simulation', 'Flight test','FontSize',14);
    xlabel('Time [s]', 'FontSize', 16);
    ylabel('\theta [deg]', 'FontSize', 16);
    
    subplot(4,1,4)
    plot(duration,qcv_sim, duration,q_meas*c/V0,'LineWidth',5);
    legend('Simulation', 'Flight test','FontSize',14);
    xlabel('Time [s]', 'FontSize', 16)
    ylabel('$q\bar{c}/V$','Interpreter','Latex', 'FontSize', 16);
    
%     % Plot eigenvalues
%     figure
%     plot(eig(As),'o')
%     set(gca, 'XAxisLocation', 'origin', 'YAxisLocation', 'origin')
%     disp(eig(As))
else
	out = lsim(sysa,[da dr], duration);
    
    beta_sim = rad2deg(out(:,1));       % Yaw angle [deg]
    phi_sim = rad2deg(out(:,2));        % Roll angle [deg]
    pb2v_sim = out(:,3);
    rb2v_sim = out(:,4);
    
    subplot(4,1,1)
    plot(duration, beta_sim,'LineWidth',5)
    legend('Simulation','FontSize',14);
    xlabel('Time [s]', 'FontSize', 16);
    ylabel('\beta [deg]', 'FontSize', 16);
    
    subplot(4,1,2)
    plot(duration,phi_sim, duration,flightdata.Ahrs1_Roll.data(is),'LineWidth',5)
    legend('Simulation', 'Flight test','FontSize',14);
    xlabel('Time [s]', 'FontSize', 16);
    ylabel('\phi [deg]', 'FontSize', 16);
    
    subplot(4,1,3)
    plot(duration,pb2v_sim, duration,p_meas*b/2/V0,'LineWidth',5)
    legend('Simulation', 'Flight test','FontSize',14);
    xlabel('Time [s]', 'FontSize', 16);
    ylabel('$\frac{p_{meas}\cdot b}{2V_{t_0}}$','Interpreter','Latex', 'FontSize', 16);
    
    subplot(4,1,4)
    plot(duration, rb2v_sim, duration,r_meas*b/2/V0,'LineWidth',5);
    legend('Simulation', 'Flight test','FontSize',14);
    xlabel('Time [s]', 'FontSize', 16)
    ylabel('$\frac{r_{meas}\cdot b}{2V_{t_0}}$','Interpreter','Latex', 'FontSize', 16);
    
%     % Plot eigenvalues
%     figure
%     plot(eig(Aa),'o')
%     set(gca, 'XAxisLocation', 'origin', 'YAxisLocation', 'origin')
%     disp(eig(Aa))
end
