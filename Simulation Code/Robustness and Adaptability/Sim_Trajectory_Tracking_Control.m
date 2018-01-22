%%
%   Tracking Control for Trajectory
%   PID Structure and PI-STR Structure
%   Nonlinear Quadrotor Model
%   By Wenchao Lei (wcleithinking@gmail.com)
%   Environment: MATLAB R2014a
%   Change variable: "controller_chosen" as 1 for the robust PID controller
%   Change variable: "controller_chosen" as 2 for the PI-STR controller
%   Change variable: "state_add_dist_aero" as 1 for air density change and
%                                             0 for none
%   Change variable: "state_add_dist_wind" as 1 for wind gusts and
%                                             0 for none
%   Change variable: "state_change_m" as 1 for mass change and
%                                             0 for none
%   Change variable: "state_change_U" as 1 for voltage change and
%                                             0 for none
%%
%   After separately running "controller_chosen = 1" and 
%   "controller_chosen = 2" in this script, we can run the script file
%   "Plot_CompareResults_PID_vs_PI_STR.m" for comparison.

%% Clear WorkSpace
clc; clear; close all;
disp(['Simulation start at ', datestr(now)]);
disp('Please do not shut down this computer !!!')
disp('Thanks!')
tic

%% Load System Parameters
Load_SystemParameters;

%% Simulation Setting
t_end = 120;
dt = 0.001;
N = floor(t_end/dt);
t_index = dt*(0:N);
% choose controller 
controller_chosen = 2;
% test robustness
state_add_dist_aero = 1;
state_add_dist_wind = 1;
A_aero_bias = 0.2;
A_wind_torque = 2e-2; A_wind_force = 5;
% test adaptability
state_change_m = 0;
state_change_U = 0;
mass_add = 0.4;
mass_remove = 0.2;
Umax_decay = 0.11/3;

%% Preallocation Memory Space
% reference signals
xi_ref = zeros(3,N+1);
dot_xi_ref = zeros(3,N+1);
ddot_xi_ref = zeros(3,N+1);
roll_ref = zeros(1,N+1);
pitch_ref = zeros(1,N+1);
yaw_ref = zeros(1,N+1);
dot_roll_ref = zeros(1,N+1);
dot_pitch_ref = zeros(1,N+1);
dot_yaw_ref = zeros(1,N+1);
y_ref = zeros(4,N+1);
dot_y_ref = zeros(4,N+1);
% ESO
hat_roll_ref = zeros(1,N+1);
hat_dot_roll_ref = zeros(1,N+1);
hat_pitch_ref = zeros(1,N+1);
hat_dot_pitch_ref = zeros(1,N+1);
% estimator
varphi1 = zeros(4,N+1);
varphi2 = zeros(4,N+1);
varphi3 = zeros(4,N+1);
varphi4 = zeros(4,N+1);
error1 = zeros(1,N+1);
error2 = zeros(1,N+1);
error3 = zeros(1,N+1);
error4 = zeros(1,N+1);
K1 = cell(1,N+1);
K2 = cell(1,N+1);
K3 = cell(1,N+1);
K4 = cell(1,N+1);
P1 = cell(1,N+1);
P2 = cell(1,N+1);
P3 = cell(1,N+1);
P4 = cell(1,N+1);
hattheta1 = zeros(4,N+1);
hattheta2 = zeros(4,N+1);
hattheta3 = zeros(4,N+1);
hattheta4 = zeros(4,N+1);
% controller
% c1
u_TEMP_c1 = zeros(4,N+1);
u_c1 = zeros(4,N+1);
% c2
u_c_c2 = zeros(4,N+1);
t0_c2 = zeros(4,N+1);
t1_c2 = zeros(4,N+1);
r1_c2 = zeros(4,N+1);
s0_c2 = zeros(4,N+1);
s1_c2 = zeros(4,N+1);
u_TEMP_c2 = zeros(4,N+1);
u_c2 = zeros(4,N+1);
% results
u_design = zeros(4,N+1);
PWM = zeros(4,N+1);
PWM_percent = zeros(4,N+1);
u_actual = zeros(4,N+1);
% motor
Umax_motor = zeros(1,N+1);
I_motor = zeros(4,N+1);
U_motor = zeros(4,N+1);
w_motor = zeros(4,N+1);
% vehicle specification
m_quad = zeros(1,N+1);
J11 = zeros(1,N+1);
J22 = zeros(1,N+1);
J33 = zeros(1,N+1);
% force and torque
T = zeros(1,N+1);
Gamma = zeros(3,N+1);
% attitude dynamics
Omega = zeros(3,N+1);
C_I2B = cell(1,N+1);
quaternion = zeros(4,N+1);
roll = zeros(1,N+1);
pitch = zeros(1,N+1);
yaw = zeros(1,N+1);
% position dynamics
f = zeros(3,N+1);
v = zeros(3,N+1);
xi = zeros(3,N+1);
% sensors
y_m = zeros(4,N+1);
dot_y_m = zeros(4,N+1);

%% Initial Settings
%% controllers' test setting
switch controller_chosen
    case 1
        h_sample = 0.005;   % 5ms
        period_t = floor(h_sample/dt);
        state_c1 = 1;
        state_c2 = 0;
    case 2
        h_sample = 0.02;    % 20ms
        period_t = floor(h_sample/dt);
        state_c1 = 0;
        state_c2 = 1;
        time_start_c2 = 0;
end
%% controllers
% for feedforward controller
PWM_min = 1000; PWM_max = 2000;
throttle = PWM_min + (PWM_max-PWM_min)*PWM_N;
% for feedback controller
umin = -[80;80;100;500];
umax =  [80;80;100;500];
% c1
kP_c1 = [10,10,20,30]';
kI_c1 = [0.2,0.2,0.4,0.6]'*1;
kD_c1 = [2,2,4,60]';
sum_error_outer_c1 = zeros(4,1);
% c2
kP_c2 = 1.0*[1;1;1;1];
kI_c2 = 0.02*[1;1;1;1];
sum_error_outer_c2 = zeros(4,1);
zeta = [0.707;0.707;0.707;0.707];
omega_n = [0.44;0.44;0.44;0.44]/h_sample;
pole_STR = [0;0;0;0];
am1 = -2*exp(-zeta.*omega_n*h_sample).*cos(omega_n.*sqrt(1-zeta.^2)*h_sample);
am2 = (exp(-zeta.*omega_n*h_sample)).^2;
ao1 = pole_STR;
%% reference trajectory
vx_d = 4;
A_vy = 60; B_vy = 00; omega_vy = 2*pi/30; phase_vy = pi;
A_vz = 10; B_vz = -20; omega_vz = 2*pi/30; phase_vz = pi/2;
A_yaw = 60/sqrt(2)*deg2rad; omega_yaw = 2*pi/20;
pole_position = -0.2;
k1_position = pole_position^2; k2_position = -2*pole_position;
%% estimators
% ESO
pole_ESO = -2;
k1_ESO = pole_ESO^2; k2_ESO = -2*pole_ESO;
hat_roll_ref(1) = 0;
hat_dot_roll_ref(1) = 0;
hat_pitch_ref(1) = 0;
hat_dot_pitch_ref(1) = 0;
% RLS-FF
a2 = exp(-tf_beta*h_sample);
a1 = -(1+a2);
b0 = tf_alpha./tf_beta*h_sample - tf_alpha./(tf_beta.^2).*(1-a2);
b1 = ( 1./tf_beta.*(1-a2) ).*( tf_alpha./tf_beta.*(1-a2) ) - a2.*b0;
percent = 0.5; Pmax = 100; ff = 0.998;
hattheta1(:,1) = percent*[a1(1),a2(1),b0(1),b1(1)];
hattheta2(:,1) = percent*[a1(2),a2(2),b0(2),b1(2)];
hattheta3(:,1) = percent*[a1(3),a2(3),b0(3),b1(3)];
hattheta4(:,1) = percent*[a1(4),a2(4),b0(4),b1(4)];
P1{1} = Pmax*eye(4);
P2{1} = Pmax*eye(4);
P3{1} = Pmax*eye(4);
P4{1} = Pmax*eye(4);
lambda = ff*ones(4,1);
%% vehicle states
% motor
Umax_motor(1) = Umax_motor_normal;
I_motor(:,1) = zeros(4,1);
w_motor(:,1) = 0.0*[w_motor_N;w_motor_N;w_motor_N;w_motor_N];
% attitude
roll(1) = 2*deg2rad;
pitch(1) = -2*deg2rad;
yaw(1) = 5*deg2rad;
Omega(:,1) = zeros(3,1);
psi_rad = yaw(1); theta_rad = pitch(1); phi_rad = roll(1);
q0 = cos(psi_rad/2)*cos(theta_rad/2)*cos(phi_rad/2)-sin(psi_rad/2)*sin(theta_rad/2)*sin(phi_rad/2);
q1 = cos(psi_rad/2)*cos(theta_rad/2)*sin(phi_rad/2)+sin(psi_rad/2)*sin(theta_rad/2)*cos(phi_rad/2);
q2 = cos(psi_rad/2)*sin(theta_rad/2)*cos(phi_rad/2)-sin(psi_rad/2)*cos(theta_rad/2)*sin(phi_rad/2);
q3 = cos(psi_rad/2)*sin(theta_rad/2)*sin(phi_rad/2)+sin(psi_rad/2)*cos(theta_rad/2)*cos(phi_rad/2);
quaternion(:,1) = [q0;q1;q2;q3];
c11 = q0^2+q1^2-q2^2-q3^2;
c12 = 2*(q1*q2+q0*q3);
c13 = 2*(q1*q3-q0*q2);
c21 = 2*(q1*q2-q0*q3);
c22 = q0^2-q1^2+q2^2-q3^2;
c23 = 2*(q2*q3+q0*q1);
c31 = 2*(q1*q3+q0*q2);
c32 = 2*(q2*q3-q0*q1);
c33 = q0^2-q1^2-q2^2+q3^2;
C_I2B{1} = [c11,c12,c13;c21,c22,c23;c31,c32,c33];
% position
xi(:,1) = [0;0;0];
v(:,1) = [0;0;0];
%% Sensors
noise_scale_rad = 0.8*deg2rad;
noise_scale_rad_per_s = 0.1*deg2rad;
noise_scale_m = 0.2;
noise_scale_m_per_s = 0.05;

%% Start Simulation
for t = 1:N+1
    %% Current Time
    t_now = dt*(t-1);
    
    if (mod(t_now,h_sample)==0)
        %% Sensors
        % Euler angles in rad
        roll_m = roll(t)+noise_scale_rad*randn;
        pitch_m = pitch(t)+noise_scale_rad*randn;
        yaw_m = yaw(t)+noise_scale_rad*randn;
        Omega_m = Omega(:,t)+noise_scale_rad_per_s*randn;
        % outputs in deg and m
        xi_m = xi(:,t)+noise_scale_m*randn;
        v_m = v(:,t)+noise_scale_m_per_s*randn;
        % results
        y_m(1,t) = roll_m*rad2deg;
        y_m(2,t) = pitch_m*rad2deg;
        y_m(3,t) = yaw_m*rad2deg;
        y_m(4,t) = xi_m(3);
        dot_y_m(1:3,t) = Omega_m*rad2deg;
        dot_y_m(4,t) = v_m(3);
        
        %% Reference Trajectory
        % position in m m/s m/s^2
        xi_ref(1,t) = vx_d*t_now;
        xi_ref(2,t) = A_vy*sin(omega_vy*t_now+phase_vy) + B_vy;
        xi_ref(3,t) = A_vz*sin(omega_vz*t_now+phase_vz) + B_vz;
        dot_xi_ref(1,t) = vx_d;
        dot_xi_ref(2,t) = omega_vy*A_vy*cos(omega_vy*t_now+phase_vy);   
        dot_xi_ref(3,t) = omega_vz*A_vz*cos(omega_vz*t_now+phase_vz);   
        ddot_xi_ref(1,t) = 0;
        ddot_xi_ref(2,t) = -omega_vy^2*A_vy*sin(omega_vy*t_now+phase_vy);   % used in ESO for attitude planner
        ddot_xi_ref(3,t) = -omega_vz^2*A_vz*sin(omega_vz*t_now+phase_vz);   % used in ESO for attitude planner
        % yaw in rad rad/s
        yaw_ref(t) = A_yaw*sin(omega_yaw*t_now) + A_yaw*cos(omega_yaw*t_now);
        dot_yaw_ref(t) = A_yaw*omega_yaw*cos(omega_yaw*t_now) - A_yaw*omega_yaw*sin(omega_yaw*t_now);
        
        %% Attitude Planner
        u_xi_x = ddot_xi_ref(1,t) - k1_position*(xi_m(1)-xi_ref(1,t)) - k2_position*(v_m(1)-dot_xi_ref(1,t));
        u_xi_y = ddot_xi_ref(2,t) - k1_position*(xi_m(2)-xi_ref(2,t)) - k2_position*(v_m(2)-dot_xi_ref(2,t));
        pitch_ref(t) = asin( u_xi_x / (-m_normal*g) );
        if t_now<=2
            roll_ref(t)  = asin( u_xi_y / (m_normal*g*cos(pitch_m)) );
        else
            roll_ref(t)  = asin( u_xi_y / (m_normal*g*cos(pitch_ref(t))) );
        end
        dot_roll_ref(t) = hat_dot_roll_ref(t);
        dot_pitch_ref(t) = hat_dot_pitch_ref(t);
        % ESO (in rad and rad/s)
        hat_roll_ref(t+1) = hat_roll_ref(t) + h_sample*( hat_dot_roll_ref(t) - k1_ESO*(hat_roll_ref(t)-roll_ref(t)) );
        hat_dot_roll_ref(t+1) = hat_dot_roll_ref(t) + h_sample*( 0 - k2_ESO*(hat_roll_ref(t)-roll_ref(t)) );
        hat_pitch_ref(t+1) = hat_pitch_ref(t) + h_sample*( hat_dot_pitch_ref(t)-k1_ESO*(hat_pitch_ref(t)-pitch_ref(t)) );
        hat_dot_pitch_ref(t+1) = hat_dot_pitch_ref(t) + h_sample*( 0 - k2_ESO*(hat_pitch_ref(t)-pitch_ref(t)) );
        % result
        y_ref(:,t) = [roll_ref(t)*rad2deg;pitch_ref(t)*rad2deg;yaw_ref(t)*rad2deg;xi_ref(3,t)];
        dot_y_ref(:,t) = [dot_roll_ref(t)*rad2deg;dot_pitch_ref(t)*rad2deg;dot_yaw_ref(t)*rad2deg;dot_xi_ref(3,t)];
        
        %% Estimators
        if state_c2 == 1
            % regressors for inner loop
            if (t_now==0)
                varphi1(:,t) = [0,0,0,0];
                varphi2(:,t) = [0,0,0,0];
                varphi3(:,t) = [0,0,0,0];
                varphi4(:,t) = [0,0,0,0];
            elseif (t_now==h_sample)
                varphi1(:,t) = [-dot_y_m(1,t-period_t),0,u_actual(1,t-period_t),0];
                varphi2(:,t) = [-dot_y_m(2,t-period_t),0,u_actual(2,t-period_t),0];
                varphi3(:,t) = [-dot_y_m(3,t-period_t),0,u_actual(3,t-period_t),0];
                varphi4(:,t) = [-dot_y_m(4,t-period_t),0,u_actual(4,t-period_t),0];
            else
                varphi1(:,t) = [-dot_y_m(1,t-period_t),-dot_y_m(1,t-2*period_t),u_actual(1,t-period_t),u_actual(1,t-2*period_t)];
                varphi2(:,t) = [-dot_y_m(2,t-period_t),-dot_y_m(2,t-2*period_t),u_actual(2,t-period_t),u_actual(2,t-2*period_t)];
                varphi3(:,t) = [-dot_y_m(3,t-period_t),-dot_y_m(3,t-2*period_t),u_actual(3,t-period_t),u_actual(3,t-2*period_t)];
                varphi4(:,t) = [-dot_y_m(4,t-period_t),-dot_y_m(4,t-2*period_t),u_actual(4,t-period_t),u_actual(4,t-2*period_t)];
            end
            % RLS-FF for roll
            error1(t) = dot_y_m(1,t) - hattheta1(:,t)'*varphi1(:,t);
            K1{t} = P1{t}*varphi1(:,t)/( lambda(1)+varphi1(:,t)'*P1{t}*varphi1(:,t) );
            hattheta1(:,t+1) = hattheta1(:,t)+K1{t}*error1(t);
            P1{t+1} = ( eye(4)-K1{t}*varphi1(:,t)' )*P1{t}/lambda(1);
            % RLS-FF for pitch
            error2(t) = dot_y_m(2,t) - hattheta2(:,t)'*varphi2(:,t);
            K2{t} = P2{t}*varphi2(:,t)/( lambda(2)+varphi2(:,t)'*P2{t}*varphi2(:,t) );
            hattheta2(:,t+1) = hattheta2(:,t)+K2{t}*error2(t);
            P2{t+1} = ( eye(4)-K2{t}*varphi2(:,t)' )*P2{t}/lambda(2);
            % RLS-FF for yaw
            error3(t) = dot_y_m(3,t) - hattheta3(:,t)'*varphi3(:,t);
            K3{t} = P3{t}*varphi3(:,t)/( lambda(3)+varphi3(:,t)'*P3{t}*varphi3(:,t) );
            hattheta3(:,t+1) = hattheta3(:,t)+K3{t}*error3(t);
            P3{t+1} = ( eye(4)-K3{t}*varphi3(:,t)' )*P3{t}/lambda(3);
            % RLS-FF for altitude
            error4(t) = dot_y_m(4,t) - hattheta4(:,t)'*varphi4(:,t);
            K4{t} = P4{t}*varphi4(:,t)/( lambda(4)+varphi4(:,t)'*P4{t}*varphi4(:,t) );
            hattheta4(:,t+1) = hattheta4(:,t)+K4{t}*error4(t);
            P4{t+1} = ( eye(4)-K4{t}*varphi4(:,t)' )*P4{t}/lambda(4);
            % estimated tf for roll
            hata1(1) = hattheta1(1,t+1);
            hata2(1) = hattheta1(2,t+1);
            hatb0(1) = hattheta1(3,t+1);
            hatb1(1) = hattheta1(4,t+1);
            % estimated tf for pitch
            hata1(2) = hattheta2(1,t+1);
            hata2(2) = hattheta2(2,t+1);
            hatb0(2) = hattheta2(3,t+1);
            hatb1(2) = hattheta2(4,t+1);
            % estimated tf for yaw
            hata1(3) = hattheta3(1,t+1);
            hata2(3) = hattheta3(2,t+1);
            hatb0(3) = hattheta3(3,t+1);
            hatb1(3) = hattheta3(4,t+1);
            % estimated tf for altitude
            hata1(4) = hattheta4(1,t+1);
            hata2(4) = hattheta4(2,t+1);
            hatb0(4) = hattheta4(3,t+1);
            hatb1(4) = hattheta4(4,t+1);
        end
        
        %% Controllers
        %% c1-PID
        if state_c1 == 1
            error_outer_c1 = y_m(:,t) - y_ref(:,t);
            error_dot_outer_c1 = dot_y_m(:,t) - dot_y_ref(:,t);
            sum_error_outer_c1 = sum_error_outer_c1 + error_outer_c1*h_sample;
            u_TEMP_c1(:,t) = ( - kP_c1.*error_outer_c1 - kI_c1.*sum_error_outer_c1 - kD_c1.*error_dot_outer_c1);
            u_c1(:,t) = min( max( u_TEMP_c1(:,t),umin ), umax);
        end
        %% c2-PI-STR
        if state_c2 == 1
            % controllers-outer loop: PI
            barW_c2 = [cos(yaw_m)*cos(pitch_m) sin(yaw_m) 0 0
                -sin(yaw_m)*cos(pitch_m) cos(yaw_m) 0 0
                sin(pitch_m) 0 1 0
                0 0 0 1];
            error_outer_c2 = y_m(:,t) - y_ref(:,t);
            sum_error_outer_c2 = sum_error_outer_c2 + error_outer_c2*h_sample;
            if t_now <= 2 % do not use the results of ESO
                u_c_c2(:,t) = barW_c2*(              0 - kP_c2.*error_outer_c2 - kI_c2.*sum_error_outer_c2 );
            else        % use the results of ESO
                u_c_c2(:,t) = barW_c2*( dot_y_ref(:,t) - kP_c2.*error_outer_c2 - kI_c2.*sum_error_outer_c2 );
            end
            % controllers-inner loop: STR
            for j = 1:4     % solve the diophantine equation: AR + BS = A_c = A_m*A_o
                t0_c2(j,t) = ( 1+am1(j)+am2(j) )/( hatb0(j)+hatb1(j) );
                t1_c2(j,t) = t0_c2(j,t)*ao1(j);
                AA_c2 = [1,hatb0(j),0;
                    hata1(j),hatb1(j),hatb0(j);
                    hata2(j),0,hatb1(j)];
                bb_c2 = [am1(j)+ao1(j)-hata1(j);
                    am2(j)+ao1(j)*am1(j)-hata2(j);
                    ao1(j)*am2(j)];
                xx_c2 = (AA_c2+0.0*eye(3))\bb_c2;
                r1_c2(j,t) = xx_c2(1);
                s0_c2(j,t) = xx_c2(2);
                s1_c2(j,t) = xx_c2(3);
            end
            if (t_now == 0)   % design regulator
                u_TEMP_c2(:,t) = t0_c2(:,t).*u_c_c2(:,t) - s0_c2(:,t).*dot_y_m(:,t);
            else
                u_TEMP_c2(:,t) = -ao1.*u_TEMP_c2(:,t-period_t) + t0_c2(:,t).*u_c_c2(:,t) + t1_c2(:,t).*u_c_c2(:,t-period_t) ...
                    - s0_c2(:,t).*dot_y_m(:,t) - s1_c2(:,t).*dot_y_m(:,t-period_t) + (ao1-r1_c2(:,t)).*u_actual(:,t-period_t);
            end
            u_c2(:,t) = min( max( u_TEMP_c2(:,t),umin ), umax);
        end
        
        %% Controller Design
        switch controller_chosen
            case 1
                u_design(:,t) = u_c1(:,t);
            case 2
                if t_now<=time_start_c2
                    u_design(:,t) = u_c1(:,t);
                else
                    u_design(:,t) = u_c2(:,t);
                end
        end
        % design PWM signals
        PWM(:,t) = u_mix*u_design(:,t) + throttle*ones(4,1);
        PWM(:,t) = min( max( PWM(:,t),PWM_min ), PWM_max);
        u_actual(:,t) = u_mix^(-1)*( PWM(:,t)-throttle*ones(4,1) );
        PWM_percent(:,t) = ( PWM(:,t)-PWM_min )/PWM_min;
        
    else
        %% Zero Order Holder
        %% reference trajectory
        % position
        xi_ref(:,t) = xi_ref(:,t-1);
        dot_xi_ref(:,t) = dot_xi_ref(:,t-1);
        ddot_xi_ref(:,t) = ddot_xi_ref(:,t-1);
        % yaw
        yaw_ref(t) = yaw_ref(t-1);
        dot_yaw_ref(t) = dot_yaw_ref(t-1);
        %% sensors
        y_m(:,t) = y_m(:,t-1);
        dot_y_m(:,t) = dot_y_m(:,t-1);
        %% attitude planner
        roll_ref(t) = roll_ref(t-1);
        pitch_ref(t) = pitch_ref(t-1);
        dot_roll_ref(t) = dot_roll_ref(t-1);
        dot_pitch_ref(t) = dot_pitch_ref(t-1);
        % ESO
        hat_roll_ref(t+1) = hat_roll_ref(t);
        hat_dot_roll_ref(t+1) = hat_dot_roll_ref(t);
        hat_pitch_ref(t+1) = hat_pitch_ref(t);
        hat_dot_pitch_ref(t+1) = hat_dot_pitch_ref(t);
        % result
        y_ref(:,t) = y_ref(:,t-1);
        dot_y_ref(:,t) = dot_y_ref(:,t-1);
        %% estimators
        varphi1(:,t) = varphi1(:,t-1);
        varphi2(:,t) = varphi2(:,t-1);
        varphi3(:,t) = varphi3(:,t-1);
        varphi4(:,t) = varphi4(:,t-1);
        error1(t) = error1(t-1);
        error2(t) = error2(t-1);
        error3(t) = error3(t-1);
        error4(t) = error4(t-1);
        K1{t} = K1{t-1};
        K2{t} = K2{t-1};
        K3{t} = K3{t-1};
        K4{t} = K4{t-1};
        hattheta1(:,t+1) = hattheta1(:,t);
        hattheta2(:,t+1) = hattheta2(:,t);
        hattheta3(:,t+1) = hattheta3(:,t);
        hattheta4(:,t+1) = hattheta4(:,t);
        P1{t+1} = P1{t};
        P2{t+1} = P2{t};
        P3{t+1} = P3{t};
        P4{t+1} = P4{t};
        %% controllers
        % c1
        u_TEMP_c1(:,t) = u_TEMP_c1(:,t-1);
        u_c1(:,t) = u_c1(:,t-1);
        % c2
        u_c_c2(:,t) =u_c_c2(:,t-1);
        t0_c2(:,t) = t0_c2(:,t-1);
        t1_c2(:,t) = t1_c2(:,t-1);
        r1_c2(:,t) = r1_c2(:,t-1);
        s0_c2(:,t) = s0_c2(:,t-1);
        s1_c2(:,t) = s1_c2(:,t-1);
        u_TEMP_c2(:,t) = u_TEMP_c2(:,t-1);
        u_c2(:,t) = u_c2(:,t-1);
        %% controller design
        u_design(:,t) = u_design(:,t-1);
        PWM(:,t) = PWM(:,t-1);
        PWM_percent(:,t) = PWM_percent(:,t-1);
        u_actual(:,t) = u_actual(:,t-1);
    end
    
    %% True System
    % vehicle specification
    if state_change_m == 1
        if t_now<=40
            m_quad(t) = m_normal;
        elseif t_now<=55
            m_quad(t) = m_normal + mass_add;
        elseif t_now<=70
            m_quad(t) = m_normal;
        elseif t_now<=85
            m_quad(t) = m_normal - mass_remove;
        else
            m_quad(t) = m_normal;
        end
    else
        m_quad(t) = m_normal;
    end
    m_CoG = m_CoG_normal + ( m_quad(t) - m_normal );
    J11(t) = m_CoG*(h_CoG^2+3*r_CoG^2)/12 + 2*m_motor*d_arm^2 + 2*(1/12)*m_motor*(3*r_motor^2+4*h_motor^2)...
        + 2*m_rotor*d_arm^2+2*(1/12*m_rotor*(2*r_rotor)^2);
    J22(t) = m_CoG*(h_CoG^2+3*r_CoG^2)/12 + 2*m_motor*d_arm^2 + 2*(1/12)*m_motor*(3*r_motor^2+4*h_motor^2)...
        + 2*m_rotor*d_arm^2+2*(1/12*m_rotor*c_rotor^2);
    J33(t) = m_CoG*r_CoG^2/2 + 4*m_motor*d_arm^2 + 4*m_motor*r_motor^2/2 + (4/12)*m_rotor*((2*r_rotor)^2+c_rotor^2)...
        + 4*m_rotor*d_arm^2;
    J12 = 0; J21 = 0;
    J13 = 2*m_motor*h_motor*d_arm + 2*m_rotor*h_rotor*d_arm; J31 = J13;
    J23 = J13; J32 = J23;
    J = [J11(t),J12,J13;J21,J22(t),J23;J31,J32,J33(t)];
    if state_add_dist_aero == 1
        kT = (1+A_aero_bias*sin(t_now))*kT_normal;
        kQ = (1+A_aero_bias*cos(t_now))*kQ_normal;
    else
        kT = kT_normal;
        kQ = kQ_normal;
    end
    % motor
    if state_change_U == 1
        Umax_motor(t) = Umax_motor_normal - Umax_decay*t_now;
    else
        Umax_motor(t) = Umax_motor_normal;
    end
    U_motor(:,t) = Umax_motor(t)*sqrt(PWM_percent(:,t));
    I_motor(:,t+1) = I_motor(:,t) + dt*( U_motor(:,t)-ke_motor*w_motor(:,t)-rm_motor*I_motor(:,t) )/L_motor;
    I_motor(:,t+1) = max(I_motor(:,t+1),0);
    w_motor(:,t+1) = w_motor(:,t) + dt*( km_motor*I_motor(:,t) - kQ*w_motor(:,t).^2)/J_motor;
    % w_rotor_mix
    w_rotor_mix = [0 -d_arm*kT 0 d_arm*kT;
        d_arm*kT 0 -d_arm*kT 0;
        kQ -kQ kQ -kQ;
        kT kT kT kT];
    Torque_and_Force = w_rotor_mix*w_motor(:,t).^2;
    if state_add_dist_wind == 1
        if (t_now>=40)&&(t_now<=80)
            Gamma(:,t) = Torque_and_Force(1:3) + A_wind_torque*cos(t_now)*ones(3,1);
            T(t) = Torque_and_Force(4) + A_wind_force*sin(t_now);
        else
            Gamma(:,t) = Torque_and_Force(1:3);
            T(t) = Torque_and_Force(4);
        end
    else
        Gamma(:,t) = Torque_and_Force(1:3);
        T(t) = Torque_and_Force(4);
    end
    % attitude
    JOmega = J*Omega(:,t);
    Gyro_Torque = [Omega(2,t)*JOmega(3)-Omega(3,t)*JOmega(2);
        Omega(3,t)*JOmega(1)-Omega(1,t)*JOmega(3);
        Omega(1,t)*JOmega(2)-Omega(2,t)*JOmega(1)];
    Omega(:,t+1) = Omega(:,t) + dt*J^(-1)*( -Gyro_Torque+Gamma(:,t) );
    Q_quaternion = [-q1 -q2 -q3;
        q0 -q3 q2;
        q3 q0 -q1;
        -q2 q1 q0];
    dot_quaternion = 1/2*Q_quaternion*Omega(:,t);
    quaternion(:,t+1) = quaternion(:,t) + dt*dot_quaternion;
    q0 = quaternion(1,t+1)/norm(quaternion(:,t+1));
    q1 = quaternion(2,t+1)/norm(quaternion(:,t+1));
    q2 = quaternion(3,t+1)/norm(quaternion(:,t+1));
    q3 = quaternion(4,t+1)/norm(quaternion(:,t+1));
    c11 = q0^2+q1^2-q2^2-q3^2;
    c12 = 2*(q1*q2+q0*q3);
    c13 = 2*(q1*q3-q0*q2);
    c21 = 2*(q1*q2-q0*q3);
    c22 = q0^2-q1^2+q2^2-q3^2;
    c23 = 2*(q2*q3+q0*q1);
    c31 = 2*(q1*q3+q0*q2);
    c32 = 2*(q2*q3-q0*q1);
    c33 = q0^2-q1^2-q2^2+q3^2;
    C_I2B{t+1} = [c11,c12,c13;c21,c22,c23;c31,c32,c33];
    roll(t+1) = atan2(-c32,c33);
    pitch(t+1) = asin(c31);
    yaw(t+1) = atan2(-c21,c11);
    % position
    f(:,t) = - C_I2B{t}'*T(t)*e3;
    v(:,t+1) = v(:,t) + dt*( g*e3 + f(:,t)/m_quad(t) );
    xi(:,t+1) = xi(:,t) + dt*v(:,t);
    
end

%% Plot
index_plot = 1:N+1;
time_plot = t_index;
% switch controller_chosen
%     case 1
%         Plot_TrackingResults_OuterLoop;
%         Plot_TrackingResults_Trajectory;
%     case 2
%         Plot_EstimationResultsofA_in_PI_STR;
%         Plot_TrackingResults_InnerLoop;
%         Plot_TrackingResults_OuterLoop;
%         Plot_TrackingResults_Trajectory;
% end

%% Save Data
switch controller_chosen
    case 1
        xi_PID = xi;
        roll_PID = roll;
        pitch_PID = pitch;
        yaw_PID = yaw;
        save data_PID.mat index_plot time_plot xi_PID roll_PID pitch_PID yaw_PID;
    case 2
        xi_PI_STR = xi;
        roll_PI_STR = roll;
        pitch_PI_STR = pitch;
        yaw_PI_STR = yaw;
        save data_PI_STR.mat index_plot time_plot xi_PI_STR roll_PI_STR pitch_PI_STR yaw_PI_STR;
end

toc