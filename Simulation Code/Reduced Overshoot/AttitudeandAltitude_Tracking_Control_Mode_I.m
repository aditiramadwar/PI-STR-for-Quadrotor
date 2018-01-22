%%
%   Tracking Control for Attitude and Altitude
%   Mode I: STR-based Single-Loop Structure
%   Nonlinear Quadrotor Model
%   By Wenchao Lei (wcleithinking@gmail.com)
%   Environment: MATLAB R2014a 
%   Change variable: "type_reference" as 1 for hovering
%   Change variable: "type_reference" as 2 for tracking sinusoidal signals

%% Clear WorkSpace
clc; clear; close all;
disp(['Simulation start at ', datestr(now)]);
disp('Please do not shut down this computer !!!')
disp('Thanks!')
tic

%% Load System Parameters
Load_SystemParameters;

%% Simulation Setting
t_end = 30;
dt = 0.001;
N = floor(t_end/dt);
t_index = dt*(0:N);
type_reference = 1;

%% Preallocation Memory Space
% reference signals
roll_ref = zeros(1,N+1);
pitch_ref = zeros(1,N+1);
yaw_ref = zeros(1,N+1);
altitude_ref = zeros(1,N+1);
dot_roll_ref = zeros(1,N+1);
dot_pitch_ref = zeros(1,N+1);
dot_yaw_ref = zeros(1,N+1);
dot_altitude_ref = zeros(1,N+1);
y_ref = zeros(4,N+1);
dot_y_ref = zeros(4,N+1);
% estimator
varphi1 = zeros(6,N+1);
varphi2 = zeros(6,N+1);
varphi3 = zeros(6,N+1);
varphi4 = zeros(6,N+1);
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
hattheta1 = zeros(6,N+1);
hattheta2 = zeros(6,N+1);
hattheta3 = zeros(6,N+1);
hattheta4 = zeros(6,N+1);
% controller
u_c = zeros(4,N+1);
t0 = zeros(4,N+1);
t1 = zeros(4,N+1);
t2 = zeros(4,N+1);
r1 = zeros(4,N+1);
r2 = zeros(4,N+1);
s0 = zeros(4,N+1);
s1 = zeros(4,N+1);
s2 = zeros(4,N+1);
u_TEMP = zeros(4,N+1);
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
% altitude dynamics
F_z = zeros(1,N+1);
v_z = zeros(1,N+1);
altitude = zeros(1,N+1);
% sensors
y_m = zeros(4,N+1);
dot_y_m = zeros(4,N+1);

%% Initial Settings
%% controllers' test setting
h_sample = 0.04;
period_t = floor(h_sample/dt);
noise_scale_rad = 0.0*deg2rad;
noise_scale_rad_per_s = 0.00*deg2rad;
noise_scale_m = 0.0;
noise_scale_m_per_s = 0.00;
%% controllers
% for feedforward controller
PWM_min = 1000; PWM_max = 2000;
throttle = PWM_min + (PWM_max-PWM_min)*PWM_N;
% for feedback controller
umin = -[80;80;100;500];
umax =  [80;80;100;500];
% desired transfer functions
am1 = zeros(4,1); am2 = zeros(4,1); am3 = zeros(4,1);
ao1 = zeros(4,1); ao2 = zeros(4,1);
zeta = 0.707; omega_n = 7.9; cc = 50;
Gdc = tf(omega_n^2*cc,[1, (2*zeta*omega_n+cc), (omega_n^2+2*zeta*omega_n*cc), cc*omega_n^2]);
Gdd = c2d(Gdc,h_sample); Gmd_den = Gdd.den{1}; pole_STR = -0.0; mu1 = pole_STR; mu2 = pole_STR;
for j = 1:4
    am1(j) = Gmd_den(2); am2(j) = Gmd_den(3); am3(j) = Gmd_den(4);
    ao1(j) = -(mu1+mu2); ao2(j) = mu1*mu2;
end
%% estimators
% RLS-FF
a1 = zeros(4,1); a2 = zeros(4,1); a3 = zeros(4,1);
b0 = zeros(4,1); b1 = zeros(4,1); b2 = zeros(4,1);
for j = 1:4
    Gc = tf(tf_beta(j),[1,tf_alpha(j),0,0]);
    Gd = c2d(Gc,h_sample);
    Gd_den = Gd.den{1};                                                                                                                                                                                                   Gd_den = Gd.den{1};
    Gd_num = Gd.num{1};
    a1(j) = Gd_den(2); a2(j) = Gd_den(3); a3(j) = Gd_den(4);
    b0(j) = Gd_num(2); b1(j) = Gd_num(3); b2(j) = Gd_num(4);
end
percent = 0.9; Pmax = 100; ff = 0.998;
hattheta1(:,1) = percent*[a1(1),a2(1),a3(1),b0(1),b1(1),b2(1)];
hattheta2(:,1) = percent*[a1(2),a2(2),a3(2),b0(2),b1(2),b2(2)];
hattheta3(:,1) = percent*[a1(3),a2(3),a3(3),b0(3),b1(3),b2(3)];
hattheta4(:,1) = percent*[a1(4),a2(4),a3(4),b0(4),b1(4),b2(4)];
P1{1} = Pmax*eye(6);
P2{1} = Pmax*eye(6);
P3{1} = Pmax*eye(6);
P4{1} = Pmax*eye(6);
lambda = ff*ones(4,1);
%% vehicle states
% motor
Umax_motor(1) = Umax_motor_normal;
I_motor(:,1) = zeros(4,1);
w_motor(:,1) = 0*[w_motor_N,w_motor_N,w_motor_N,w_motor_N];
% attitude
roll(1) = 2*deg2rad;
pitch(1) = -2*deg2rad;
yaw(1) = 5*deg2rad;
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
Omega(:,1) = zeros(3,1);
altitude(1) = 0;
v_z(1) = 0;

%% Start Simulation
for t = 1:N+1
    %% Current Time
    t_now = dt*(t-1);
    
    if (mod(t_now,h_sample)==0)
        %% Sensors
        % attitude in rad
        roll_m = roll(t) + noise_scale_rad*randn;
        pitch_m = pitch(t) + noise_scale_rad*randn;
        yaw_m = yaw(t) + noise_scale_rad*randn;
        Omega_m = Omega(:,t) + noise_scale_rad_per_s*randn;
        % altitude in m
        altitude_m = altitude(t) + noise_scale_m*randn;
        v_z_m = v_z(t) + noise_scale_m_per_s*randn;
        % results
        y_m(1,t) = roll_m*rad2deg;
        y_m(2,t) = pitch_m*rad2deg;
        y_m(3,t) = yaw_m*rad2deg;
        y_m(4,t) = altitude_m;
        dot_y_m(1:3,t) = Omega_m*rad2deg;
        dot_y_m(4,t) = v_z_m;
        
        %% Reference Trajectory
        switch type_reference
            case 1
                roll_ref(t) = 0;
                pitch_ref(t) = 0;
                yaw_ref(t) = 0;
                altitude_ref(t) = 6;
                dot_roll_ref(t) = 0;
                dot_pitch_ref(t) = 0;
                dot_yaw_ref(t) = 0;
                dot_altitude_ref(t) = 0;
            case 2
                roll_ref(t) = 5*deg2rad*sin(2*t_now);
                pitch_ref(t) = 5*deg2rad*cos(2*t_now);
                yaw_ref(t) = 10*deg2rad*sin(2*t_now);
                altitude_ref(t) = 5+1*sin(t_now);
                dot_roll_ref(t) = 5*deg2rad*2*cos(2*t_now);
                dot_pitch_ref(t) = -5*deg2rad*2*sin(2*t_now);
                dot_yaw_ref(t) = 10*deg2rad*2*cos(2*t_now);
                dot_altitude_ref(t) = 1*cos(t_now);
        end
        % result in deg and m
        y_ref(:,t) = [roll_ref(t)*rad2deg;pitch_ref(t)*rad2deg;yaw_ref(t)*rad2deg;altitude_ref(t)];
        dot_y_ref(:,t) = [dot_roll_ref(t)*rad2deg;dot_pitch_ref(t)*rad2deg;dot_yaw_ref(t)*rad2deg;dot_altitude_ref(t)];
        
        %% Estimators
        % regressors
        if (t_now == 0)
            varphi1(:,t) = [0,0,0,0,0,0];
            varphi2(:,t) = [0,0,0,0,0,0];
            varphi3(:,t) = [0,0,0,0,0,0];
            varphi4(:,t) = [0,0,0,0,0,0];
        elseif (t_now==h_sample)
            varphi1(:,t) = [-y_m(1,t-period_t),0,0,u_actual(1,t-period_t),0,0];
            varphi2(:,t) = [-y_m(2,t-period_t),0,0,u_actual(2,t-period_t),0,0];
            varphi3(:,t) = [-y_m(3,t-period_t),0,0,u_actual(3,t-period_t),0,0];
            varphi4(:,t) = [-y_m(4,t-period_t),0,0,u_actual(4,t-period_t),0,0];
        elseif (t_now==2*h_sample)
            varphi1(:,t) = [-y_m(1,t-period_t),-y_m(1,t-2*period_t),0,u_actual(1,t-period_t),u_actual(1,t-2*period_t),0];
            varphi2(:,t) = [-y_m(2,t-period_t),-y_m(2,t-2*period_t),0,u_actual(2,t-period_t),u_actual(2,t-2*period_t),0];
            varphi3(:,t) = [-y_m(3,t-period_t),-y_m(3,t-2*period_t),0,u_actual(3,t-period_t),u_actual(3,t-2*period_t),0];
            varphi4(:,t) = [-y_m(4,t-period_t),-y_m(4,t-2*period_t),0,u_actual(4,t-period_t),u_actual(4,t-2*period_t),0];
        else
            varphi1(:,t) = [-y_m(1,t-period_t),-y_m(1,t-2*period_t),-y_m(1,t-3*period_t),u_actual(1,t-period_t),u_actual(1,t-2*period_t),u_actual(1,t-3*period_t)];
            varphi2(:,t) = [-y_m(2,t-period_t),-y_m(2,t-2*period_t),-y_m(2,t-3*period_t),u_actual(2,t-period_t),u_actual(2,t-2*period_t),u_actual(2,t-3*period_t)];
            varphi3(:,t) = [-y_m(3,t-period_t),-y_m(3,t-2*period_t),-y_m(3,t-3*period_t),u_actual(3,t-period_t),u_actual(3,t-2*period_t),u_actual(3,t-3*period_t)];
            varphi4(:,t) = [-y_m(4,t-period_t),-y_m(4,t-2*period_t),-y_m(4,t-3*period_t),u_actual(4,t-period_t),u_actual(4,t-2*period_t),u_actual(4,t-3*period_t)];
        end
        % RLS-FF for roll
        error1(t) = y_m(1,t) - hattheta1(:,t)'*varphi1(:,t);
        K1{t} = P1{t}*varphi1(:,t)/( lambda(1)+varphi1(:,t)'*P1{t}*varphi1(:,t) );
        hattheta1(:,t+1) = hattheta1(:,t)+K1{t}*error1(t);
        P1{t+1} = ( eye(6)-K1{t}*varphi1(:,t)' )*P1{t}/lambda(1);
        % RLS-FF for pitch
        error2(t) = y_m(2,t) - hattheta2(:,t)'*varphi2(:,t);
        K2{t} = P2{t}*varphi2(:,t)/( lambda(2)+varphi2(:,t)'*P2{t}*varphi2(:,t) );
        hattheta2(:,t+1) = hattheta2(:,t)+K2{t}*error2(t);
        P2{t+1} = ( eye(6)-K2{t}*varphi2(:,t)' )*P2{t}/lambda(2);
        % RLS-FF for yaw
        error3(t) = y_m(3,t) - hattheta3(:,t)'*varphi3(:,t);
        K3{t} = P3{t}*varphi3(:,t)/( lambda(3)+varphi3(:,t)'*P3{t}*varphi3(:,t) );
        hattheta3(:,t+1) = hattheta3(:,t)+K3{t}*error3(t);
        P3{t+1} = ( eye(6)-K3{t}*varphi3(:,t)' )*P3{t}/lambda(3);
        % RLS-FF for altitude
        error4(t) = y_m(4,t) - hattheta4(:,t)'*varphi4(:,t);
        K4{t} = P4{t}*varphi4(:,t)/( lambda(4)+varphi4(:,t)'*P4{t}*varphi4(:,t) );
        hattheta4(:,t+1) = hattheta4(:,t)+K4{t}*error4(t);
        P4{t+1} = ( eye(6)-K4{t}*varphi4(:,t)' )*P4{t}/lambda(4);
        % estimated tf for roll
        hata1(1) = hattheta1(1,t+1);
        hata2(1) = hattheta1(2,t+1);
        hata3(1) = hattheta1(3,t+1);
        hatb0(1) = hattheta1(4,t+1);
        hatb1(1) = hattheta1(5,t+1);
        hatb2(1) = hattheta1(6,t+1);
        % estimated tf for pitch
        hata1(2) = hattheta2(1,t+1);
        hata2(2) = hattheta2(2,t+1);
        hata3(2) = hattheta2(3,t+1);
        hatb0(2) = hattheta2(4,t+1);
        hatb1(2) = hattheta2(5,t+1);
        hatb2(2) = hattheta2(6,t+1);
        % estimated tf for yaw
        hata1(3) = hattheta3(1,t+1);
        hata2(3) = hattheta3(2,t+1);
        hata3(3) = hattheta3(3,t+1);
        hatb0(3) = hattheta3(4,t+1);
        hatb1(3) = hattheta3(5,t+1);
        hatb2(3) = hattheta3(6,t+1);
        % estimated tf for altitude
        hata1(4) = hattheta4(1,t+1);
        hata2(4) = hattheta4(2,t+1);
        hata3(4) = hattheta4(3,t+1);
        hatb0(4) = hattheta4(4,t+1);
        hatb1(4) = hattheta4(5,t+1);
        hatb2(4) = hattheta4(6,t+1);
        
        %% Controller
        u_c(:,t) = y_ref(:,t);
        % solve diophantine equation: AR + BS = A_c = A_m*A_o
        for j = 1:4
            t0(j,t) = ( 1+am1(j)+am2(j)+am3(j) )/( hatb0(j)+hatb1(j)+hatb2(j) );
            t1(j,t) = t0(j,t)*ao1(j);
            t2(j,t) = t0(j,t)*ao2(j);
            AA = [1,0,hatb0(j),0,0;
                hata1(j),1,hatb1(j),hatb0(j),0;
                hata2(j),hata1(j),hatb2(j),hatb1(j),hatb0(j);
                hata3(j),hata2(j),0,hatb2(j),hatb1(j);
                0,hata3(j),0,0,hatb2(j)];
            bb = [am1(j)+ao1(j)-hata1(j);
                am2(j)+ao1(j)*am1(j)+ao2(j)-hata2(j);
                am3(j)+ao1(j)*am2(j)+ao2(j)*am1(j)-hata3(j);
                ao1(j)*am3(j)+ao2(j)*am2(j);
                ao2(j)*am3(j)];
            xx = AA\bb;
            r1(j,t) = xx(1);
            r2(j,t) = xx(2);
            s0(j,t) = xx(3);
            s1(j,t) = xx(4);
            s2(j,t) = xx(5);
        end
        % design referenced controllers according to diophantine equation
        for j = 1:4
            if (t_now == 0)
                u_TEMP(j,t) = t0(j,t)*u_c(j,t) - s0(j,t)*y_m(j,t);
            elseif (t_now==h_sample)
                u_TEMP(j,t) = -ao1(j)*u_TEMP(j,t-period_t)...
                    + t0(j,t)*u_c(j,t) + t1(j,t)*u_c(j,t-period_t)...
                    - s0(j,t)*y_m(j,t) - s1(j,t)*y_m(j,t-period_t)...
                    + (ao1(j)-r1(j,t))*u_actual(j,t-period_t);
            else
                u_TEMP(j,t) = -ao1(j)*u_TEMP(j,t-period_t)-ao2(j)*u_TEMP(j,t-period_t)...
                    + t0(j,t)*u_c(j,t) + t1(j,t)*u_c(j,t-period_t) + t2(j,t)*u_c(j,t-2*period_t)...
                    - s0(j,t)*y_m(j,t) - s1(j,t)*y_m(j,t-period_t) - s2(j,t)*y_m(j,t-2*period_t)...
                    + (ao1(j)-r1(j,t))*u_actual(j,t-period_t) + (ao2(j)-r2(j,t))*u_actual(j,t-2*period_t);
            end
        end
        u_design(:,t) = min( max( u_TEMP(:,t),umin ), umax);
        % design PWM signals
        PWM(:,t) = u_mix*u_design(:,t) + throttle*ones(4,1);
        PWM(:,t) = min( max( PWM(:,t),PWM_min ), PWM_max);
        u_actual(:,t) = u_mix^(-1)*( PWM(:,t)-throttle*ones(4,1) );
        PWM_percent(:,t) = ( PWM(:,t)-PWM_min )/PWM_min;
        
    else
        %% Zero Order Holder
        % reference trajectory
        roll_ref(t) = roll_ref(t-1);
        pitch_ref(t) = pitch_ref(t-1);
        yaw_ref(t) = yaw_ref(t-1);
        altitude_ref(t) = altitude_ref(t-1);
        dot_roll_ref(t) = dot_roll_ref(t-1);
        dot_pitch_ref(t) = dot_pitch_ref(t-1);
        dot_yaw_ref(t) = dot_yaw_ref(t-1);
        dot_altitude_ref(t) = dot_altitude_ref(t-1);
        % sensors
        y_m(:,t) = y_m(:,t-1);
        dot_y_m(:,t) = dot_y_m(:,t-1);
        % estimators
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
        % controllers
        u_design(:,t) = u_design(:,t-1);
        PWM(:,t) = PWM(:,t-1);
        PWM_percent(:,t) = PWM_percent(:,t-1);
        u_actual(:,t) = u_actual(:,t-1);
    end
    
    %% True System
    kT = kT_normal;
    kQ = kQ_normal;
    m_quad(t) = m_normal;
    % motor
    Umax_motor(t) = Umax_motor_normal;
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
    Gamma(:,t) = Torque_and_Force(1:3);
    T(t) = Torque_and_Force(4);
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
    % altitude
    F_z(t) = m_quad(t)*g - T(t)*cos(roll(t))*cos(pitch(t));
    v_z(t+1) = v_z(t) + dt*F_z(t)/m_quad(t);
    altitude(t+1) = altitude(t) + dt*v_z(t);
    
end
toc
%% Plot
index_plot = 1:N+1;
time_plot = t_index;
Plot_AttitudeandAltitude_TrackingResults;