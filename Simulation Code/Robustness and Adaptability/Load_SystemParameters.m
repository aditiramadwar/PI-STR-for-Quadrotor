%% 
% Frame: DJI F450.
% Rotor: APC 1047.
% Motor: SunnySky A2212 KV980 
% The data of motor comes from website (the measured data of XXD A2212).
% Variables "r_CoG" and "h_CoG": through intuition.
% Note that "r_CoG" influences inertia matrix "J" significantly.
% We choose "r_CoG = d_arm/2".

%% Auxiliary Variables
rho = 1.205;    % air density
g = 9.81;       % gravitational acceleration 
e3 = [0;0;1];   % unit vector in z-direction
rad2deg = 180/pi;
deg2rad = pi/180;
m2cm = 100;
cm2m = 0.01;

%% Characteristic Parameters
% arm
d_arm = 0.225;          % off-line measurement
% CoG
m_CoG_normal = 0.709;   % off-line measurement
r_CoG = d_arm*0.5;      % through intuition
h_CoG = 0.05;           % through intuition
% rotor
m_rotor = 0.013;        % off-line measurement
r_rotor = 0.120;        % off-line measurement (or use 0.127)
c_rotor = 0.025;        % off-line measurement
h_rotor = 0.045;        % off-line measurement
theta_tw = 15*pi/180;   % adjust to get "kT_normal = 1.508e-5" (fitted by measurement data)
Cl_alpha = 3.320;       % adjust to get "kT_normal = 1.508e-5" (fitted by measurement data)
Cd_zero = 0.220;        % adjust to get "kT_normal = 1.508e-5" (fitted by measurement data)
kT_normal = 1/3*rho*c_rotor*r_rotor^3*Cl_alpha*theta_tw;    % see my previous work published in IET Control Theory & Application
kH_normal = 1/2*rho*c_rotor*r_rotor^2*Cd_zero;              % see my previous work published in IET Control Theory & Application
kQ_normal = 1/4*rho*c_rotor*r_rotor^4*Cd_zero;              % see my previous work published in IET Control Theory & Application
% motor
m_motor = 0.056;        % off-line measurement
r_motor = 0.027/2;      % off-line measurement
h_motor = 0.03;         % off-line measurement
L_motor = 15e-5;        % from website
ke_motor = 0.008286;    % from curve fitting: U = ke*varpi+R*I
rm_motor = 0.3774;       % from curve fitting: U = ke*varpi+R*I
km_motor = 0.014490;    % from curve fitting: P = kq*I*varpi
J_motor = 1.25e-5;      % from website (influence the response time)
Umax_motor_normal = 11.1;   % off-line measurement

%% Mass and Inertia 
% calculated according to \cite{Bangura2012Nonlinear}
m_normal = m_CoG_normal + 4*m_rotor + 4*m_motor; 
Ixx = m_CoG_normal*(h_CoG^2+3*r_CoG^2)/12 + 2*m_motor*d_arm^2 + 2*(1/12)*m_motor*(3*r_motor^2+4*h_motor^2) + 2*m_rotor*d_arm^2+2*(1/12*m_rotor*(2*r_rotor)^2);
Iyy = m_CoG_normal*(h_CoG^2+3*r_CoG^2)/12 + 2*m_motor*d_arm^2 + 2*(1/12)*m_motor*(3*r_motor^2+4*h_motor^2) + 2*m_rotor*d_arm^2+2*(1/12*m_rotor*c_rotor^2);
Izz = m_CoG_normal*r_CoG^2/2 + 4*m_motor*d_arm^2 + 4*m_motor*r_motor^2/2 + (4/12)*m_rotor*((2*r_rotor)^2+c_rotor^2) + 4*m_rotor*d_arm^2;
Ixy = 0; 
Iyx = 0;
Ixz = 2*m_motor*h_motor*d_arm + 2*m_rotor*h_rotor*d_arm; 
Izx = Ixz;
Iyz = Ixz; 
Izy = Iyz;
J = [Ixx,Ixy,Ixz;Iyx,Iyy,Iyz;Izx,Izy,Izz];

%% Additional Information
% obtained by derivation
w_motor_N = sqrt((m_normal*g)/(4*kT_normal));
U_N = ( kQ_normal*w_motor_N^2 + km_motor*ke_motor/rm_motor*w_motor_N )*rm_motor/km_motor;
PWM_N = (U_N/Umax_motor_normal)^2;
tf_beta    = (2*kQ_normal*w_motor_N+km_motor*ke_motor/rm_motor)/J_motor*ones(4,1);
tf_alpha = [2*km_motor*w_motor_N*d_arm*kT_normal/rm_motor/Ixx/J_motor*Umax_motor_normal/250/2*rad2deg;
    2*km_motor*w_motor_N*d_arm*kT_normal/rm_motor/Iyy/J_motor*Umax_motor_normal/250/2*rad2deg;
    2*km_motor*w_motor_N*kQ_normal/rm_motor/Izz/J_motor*Umax_motor_normal/250*rad2deg;
    2*km_motor*w_motor_N*kT_normal/rm_motor/m_normal/J_motor*Umax_motor_normal/250];

%% Mix Matrices
w_mix = [0 -d_arm*kT_normal 0 d_arm*kT_normal;
    d_arm*kT_normal 0 -d_arm*kT_normal 0;
    kQ_normal -kQ_normal kQ_normal -kQ_normal;
    kT_normal kT_normal kT_normal kT_normal];
u_mix = [0  1  1 -1;
        -1  0 -1 -1;
         0 -1  1 -1;
         1  0 -1 -1];