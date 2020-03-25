ft2m = 0.3048;
lbs2kg = 0.4536;
kts2mps = 0.5144;
in2m = 0.0254;

% For test
whichone = 'test';
m_fuel = 4100*lbs2kg;       % Fuel mass [lbs]->[kg]

% % For reference
% whichone = 'reference';
% m_fuel = 4050*lbs2kg;       % Fuel mass [lbs]->[kg]

load(strcat(whichone,'_payload.mat'));
m_PL = sum(PL.data(:,1));   % Payload mass [kg]
% Shift of cg, 3L from 288 to 134
x_shift = 134-288;          % 3L moves forward to cockpit [in]

load('fuel_moment.mat');
fuel_moment.data(:,1) = fuel_moment.data(:,1)*lbs2kg;       % Fuel mass [lbs]->[kg]
fuel_moment.data(:,2) = fuel_moment.data(:,2)*lbs2kg;       % Fuel moment [in*lbs]-[in*kg]
linear_fuel = polyfit(fuel_moment.data(:,1), fuel_moment.data(:,2), 1); % Input [kg], output [in*kg]

lambda = -6.5e-3;   % [k/m]
T0 = 273.15+15;     % [K]
p0 = 101325;        % [Pa]
rho0 = 1.225;       % [kg/m^3]
gamma = 1.4;
R = 287.05;         % [J/kg/K]
g = 9.81;           % [m/s^2]

S_wing = 30;                % Total area of the wing [m^2]
b_wing = 15.911;            % wing span [m]
A = b_wing^2/S_wing;        % wing aspect ratio
MAC = 2.0569;               % mean aerodynamic cord [m]
m_OE = 9165*lbs2kg;         % Operational empty mass [lbs]->[kg]
x_cg_OE = 291.65;           % cg location of empty a/c [in], assume to be 0.25c
LEMAC = x_cg_OE*in2m-MAC/4; % Leading edge of MAC from datum
m_init = m_OE+m_PL+m_fuel;  % Initial mass of a/c+fuel+pl [kg]
D_engine = 27*in2m;         % Engine diameter [m]
CmTc = -0.0064;             % Thrust moment arm
Ws = 60500;                 % Standard a/c weight [N]
FLs = 0.048;                % Standard fuel flow [kg/s]

% Get timestamps for trim and shift of cg
load(strcat(whichone, '_data.mat'));
dt = 0.1;
load(strcat(whichone, '_trim.mat'));
load(strcat(whichone, '_shift_cg.mat'));
i_trim = int32((postdata_trim.data(:,1)-flightdata.time.data(1))/dt+1);
i_shift = int32((shift_cg.data(:,1)-flightdata.time.data(1))/dt+1);


% % Generate a matlab.dat for thrust.exe
% % Assume engine is aligned with a/c body x-axis
% % The intake air is alpha to the engine, so V_intake=TAS*cos(alpha)
% hp = flightdata.Dadc1_alt.data(i_trim)*ft2m;                % Pressure Altitude [ft]->[m]
% alpha = deg2rad(flightdata.vane_AOA.data(i_trim));          % AOA [rad]
% mach = flightdata.Dadc1_mach.data(i_trim).*cos(alpha);      % Mach axially to engine
% T_static = 273.15+flightdata.Dadc1_sat.data(i_trim);        % Static temperature [C]->[K]
% T_ISA = T0 + (lambda*hp);                                   % ISA temperature at hp [K]
% dT = T_static - T_ISA;
% % Measurement
% FFL = flightdata.lh_engine_FMF.data(i_trim)*(lbs2kg/3600);  % Left fuel flow [lbs/hr]->[kg/s]
% FFR = flightdata.rh_engine_FMF.data(i_trim)*(lbs2kg/3600);  % Right fuel flow [lbs/hr]->[kg/s]
% % % Standard
% % FFL = FLs*ones(length(i_trim),1);                           % Left standard fuel flow [kg/s]
% % FFR = FLs*ones(length(i_trim),1);                           % Right standard fuel flow [kg/s]
% thrusttable = [round(hp,4) round(mach,4) round(dT,4) round(FFL,4) round(FFR,4)];
% writematrix(thrusttable, 'matlab.dat', 'Delimiter', ' ');


% Weight
ful = flightdata.lh_engine_FU.data(i_trim)*lbs2kg;      % Fuel used of left engine [lbs]->[kg]
fur = flightdata.rh_engine_FU.data(i_trim)*lbs2kg;      % Fuel used of right engine [lbs]->[kg]
W_tot = (m_init-(ful+fur))*g;                           % Total weight at each moment [N]
% Air
TAS = flightdata.Dadc1_tas.data(i_trim)*kts2mps;        % True airspeed [knots]->[m/s]
hp = flightdata.Dadc1_alt.data(i_trim)*ft2m;            % Pressure altitude [ft]->[m]
p_static = p0*(1+lambda*hp/T0).^(-g/lambda/R);          % Static pressure [Pa]
T_static = 273.15+flightdata.Dadc1_sat.data(i_trim);    % Static temperature [C]->[K]
rho = p_static./T_static/R;                             % True air density [kg/m^3]
% Measured thrust and thrust coefficients, total thrust/single engine
thrust = importdata(strcat(whichone, '_trim_thrust.dat'));
thrust_tot = sum(thrust, 2);                            % Total thrust by both engines [N]
Tc = thrust_tot./(0.5*rho.*TAS.^2*D_engine^2);          % Thrust coefficients
% Compute -Cma/Cmd
delta_e = flightdata.delta_e.data(i_trim);              % Deflection of elevator [deg]
alpha = flightdata.vane_AOA.data(i_trim);               % AOA [deg]
trims = polyfit(alpha, delta_e, 1);                     % trims(1)=ddelta/dalpha=-Cma/Cmd
% Reduce equivalent airspeed
EAS = TAS.*sqrt(rho/rho0);                              % Equivalent airspeed [m/s]
EAS_reduced = EAS.*sqrt(Ws./W_tot);                     % Reduced equivalent airspeed [m/s]
EAS4plot = min(EAS_reduced):0.1:max(EAS_reduced);       % Only for plot
% Standard thrust coefficients, total thrust/single engine
standard_thrust = importdata(strcat(whichone, '_trim_thrust_standard.dat'));
standard_thrust_tot = sum(standard_thrust, 2);                      % Total thrust by both engines [N]
Tcs = standard_thrust_tot./(0.5*rho.*EAS_reduced.^2*D_engine^2);    % Standard thrust coefficients
% Elevator control force
Fe_meas = flightdata.column_fe.data(i_trim);                        % Measured control force [N]
Fe_reduced = Fe_meas*Ws./W_tot;                                     % Reduced control force [N]
mean_dte = round(mean(flightdata.elevator_dte.data(i_trim)),1);     % Almost constant deflection of trim tab [deg]
cg_trim = ones(length(i_trim),1);
mf = m_fuel-(ful+fur);
for i=1:length(i_trim)
    cg_trim(i)=(sum(PL.data(:,1).*PL.data(:,2)) + mf(i)*linear_fuel(1)+linear_fuel(2) + m_OE*x_cg_OE)/(W_tot(i)/g)*in2m;
end
meancg = round((mean(cg_trim)-LEMAC)/MAC,3);
dcg = max(cg_trim)-min(cg_trim);
forcecurve = polyfit(EAS_reduced.^2, Fe_reduced, 1);   	% Fe is proportional to V^2
% Plot control force curve
figure
ax_reduced_Fe_EAS=subplot(1,1,1);
scatter(EAS_reduced, Fe_reduced); hold on;
plot(EAS4plot, forcecurve(1).*EAS4plot.^2+forcecurve(2), '-r');
plot(0:0.1:min(EAS_reduced), forcecurve(1).*(0:0.1:min(EAS_reduced)).^2+forcecurve(2), '--r');
text(20,-10,['$\bar{x}_{cg}: ' num2str(meancg) '\bar{c}$'], 'Interpreter','Latex', 'FontSize', 16);
text(20,10,['$\delta_{t_e}: ' num2str(mean_dte) '^\circ$'], 'Interpreter','Latex', 'FontSize', 16);
ax_reduced_Fe_EAS.YDir = 'reverse';
xlabel('$$\tilde{V}_e [m/s]$$', 'Interpreter', 'LaTeX'); ylabel('F^*_e [N]');


% Determine Cmd and Cma by shifting cg
delta_e = flightdata.delta_e.data(i_shift);             % Deflection of the elevator before and after shifting cg
ful = flightdata.lh_engine_FU.data(i_shift)*lbs2kg;     % Fuel used of left engine [lbs]->[kg]
fur = flightdata.rh_engine_FU.data(i_shift)*lbs2kg;     % Fuel used of right engine [lbs]->[kg]
m_fuel_cgshift = m_fuel-(ful+fur);                      % Fuel remaining before and after shifting cg [kg]
m_tot_cgshift = m_OE + m_PL + m_fuel_cgshift;           % Total mass before and after shifting cg [kg]
% Compute cg location before and after of shifting cg
x_cg_before = (sum(PL.data(:,1).*PL.data(:,2)) + m_fuel_cgshift(1)*linear_fuel(1)+linear_fuel(2) + m_OE*x_cg_OE)/(m_tot_cgshift(1));
PL.data(8,2) = PL.data(8,2)+x_shift;                    % Shift forward to cockpit
x_cg_after = (sum(PL.data(:,1).*PL.data(:,2)) + m_fuel_cgshift(2)*linear_fuel(1)+linear_fuel(2) + m_OE*x_cg_OE)/(m_tot_cgshift(2));
dx_cg = (x_cg_after-x_cg_before)*in2m;                  % cg shift [in]->[m]
% Air
TAS = flightdata.Dadc1_tas.data(i_shift)*kts2mps;       % True airspeed [knots]->[m/s]
hp = flightdata.Dadc1_alt.data(i_shift)*ft2m;           % Pressure altitude [ft]->[m]
p_static = p0*(1+lambda*hp/T0).^(-g/lambda/R);          % Static pressure [Pa]
T_static = 273.15+flightdata.Dadc1_sat.data(i_shift);   % Static temperature [C]->[K]
rho = p_static./T_static/R;                             % Air density [kg/m^3]
% Coefficients
CN = m_tot_cgshift*g./(0.5*rho.*TAS.^2*S_wing);         % Normal force coefficient, two values almost equal, makes sense
Cmd = -1/(delta_e(2)-delta_e(1))*mean(CN)*dx_cg/MAC;    % [/deg]
Cma = -Cmd*trims(1);                                    % [/deg]
disp([round(Cmd,4) round(Cma,4)])

% Reduced delta_e
delta_e_meas = flightdata.delta_e.data(i_trim);             % Measured deflection of elevator [deg]
delta_e_reduced = delta_e_meas - CmTc/Cmd*(Tcs-Tc);         % Reduced deflection of elevator [deg]
trimcurve = polyfit(EAS_reduced.^-2, delta_e_reduced, 1);   % de proportional to V^-2
% Plot trim curve
figure
ax_reduced_de_EAS=subplot(1,1,1);
scatter(EAS_reduced, delta_e_reduced); hold on;
plot(EAS4plot, trimcurve(1)./EAS4plot.^2+trimcurve(2));
ax_reduced_de_EAS.YDir = 'reverse';
xlabel('$\tilde{V}_e [m/s]$', 'Interpreter', 'LaTeX'); ylabel('\delta^*_e [deg]');
