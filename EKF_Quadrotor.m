%% Clear past plots, variables and commands
close all; clear all; clc;

% Load data 
load 'data.mat'; s

% acx = x-axis accelerometer reading
% acy = y-axis accelerometer reading
% acz = z-axis accelerometer reading

% phi = Roll angle computed by the drone's on-board computer
% tht = Pitch angle computed by the drone's on-board computer
% psi = Yaw angle computed by the drone's on-board computer 

% fix = GPS position fix signal 
% eph = GPS horizontal variance 
% epv = GPS vertical variance 
% lat = GPS Latitude
% lon = GPS Longitude
% alt = GPS altitude
% gps_nSat = Number of GPS satellites
% 
% out1 = Motor 1 signal
% out2 = Motor 2 signal

% out3 = Motor 3 signal
% out4 = Motor 4 signal

%% Accelerometer plot
figure; set(gcf,'numbertitle','off','name','Acceleration');  
subplot(3,1,1); plot(t, acx, 'b'); ylim([-2 2]); ylabel('acx (m/s^2)'); grid on; 
subplot(3,1,2); plot(t, acy, 'b'); ylim([-2 2]); ylabel('acy (m/s^2)'); grid on; 
subplot(3,1,3); plot(t, acz, 'b'); ylabel('acz (m/s^2)'); grid on; 

%% Euler angles plot
figure; set(gcf,'numbertitle','off','name','Euler Angles');  
subplot(3,1,1); plot(t, rad2deg(phi), 'b'); ylabel('Roll (degree)'); grid on; 
subplot(3,1,2); plot(t, rad2deg(tht), 'b'); ylabel('Pitch (degree)'); grid on; 
subplot(3,1,3); plot(t, rad2deg(psi), 'b'); ylabel('Yaw (degree)'); grid on; 

%% GPS plot
figure; set(gcf,'numbertitle','off','name','GPS');  
subplot(3,2,1); plot(t, lon); ylabel('Longitude'); grid on;
subplot(3,2,3); plot(t, lat); ylabel('Latitude'); grid on;
subplot(3,2,5); plot(t, alt); ylabel('Altitude'); grid on; xlabel('time (s)');

subplot(3,2,2); plot(t, gps_nSat, '.'); ylabel('Sat'); grid on;
subplot(3,2,4); plot(t, eph); ylabel('Eph'); grid on; ylim([0 5]);
subplot(3,2,6); plot(t, epv); ylabel('Epv'); grid on; ylim([0 5]);

%% Motor signal plot
figure; set(gcf,'numbertitle','off','name','Motor Signal');  
hold on;
plot(t,out1,'r');
plot(t,out2,'g');
plot(t,out3,'b');
plot(t,out4,'y');
legend('Motor1','Motor2','Motor3','Motor4'); 
ylabel('Motor inputs'); xlabel('time (s)'); ylim([1000 2000]); grid on;


%%%%%%%%%%%%%%%%%%%%%% Your own coding work start from here %%%%%%%%%%%%%%%%%%%%%%%%%

%% Convert GPS raw measurements to local NED position values

clc;

% Initializing Parameters
a = 6378137;
b = 6356752.342;
e2 = 1 - ((b^2)/(a^2));

% Calculating Prime vertical radius N. N will be nx1 array
N = (a./(sqrt(1 - e2*(sin(deg2rad(lat)).^2)))); 

% Conversion from Geodetic to ECEF
xe = (N + alt) .* cos(deg2rad(lat)) .* cos(deg2rad(lon));
ye = (N + alt) .* cos(deg2rad(lat)) .* sin(deg2rad(lon));
ze = ((b^2/a^2)*N + alt) .* sin(deg2rad(lat));


% Conversion from ECEF to NED

% Initializing first data points i.e Xe_0, Ye_0 and Ze_0 
xe_ref = xe(1);
ye_ref = ye(1);
ze_ref = ze(1);

% Initizaling nx1 arrays to store NED co-ordinates after conversion 
x_ned = zeros(size(xe));
y_ned = zeros(size(ye));
z_ned = zeros(size(ze));

for i = 1 : length(xe)
    
    lon_ned = deg2rad(lon(i));
    lat_ned = deg2rad(lat(i));

    R = [-sin(lat_ned)*cos(lon_ned), -sin(lon_ned), -cos(lat_ned)*cos(lon_ned);
         -sin(lat_ned)*sin(lon_ned),  cos(lon_ned), -cos(lat_ned)*sin(lon_ned);
         cos(lat_ned), 0, -sin(lat_ned)];

    ned = R' * ([xe(i); ye(i); ze(i)] - [xe_ref; ye_ref; ze_ref]);
    x_ned(i) = ned(1);
    y_ned(i) = ned(2);
    z_ned(i) = ned(3);

end

%% Implement EKF to estimate NED position and velocity
clc;

% Initializng t_min and t_max
t_min = 1444.66 - 600;
t_max = 1688.78 + 300;

% Get the approximate indices for t_min and t_max from t (nx1 array)
t_min_ind = find(t >= t_min, 1);
t_max_ind = find(t >= t_max, 1);

% Initializing parameters for EKF
R_cov = eye(3) * 1;                                 % Measurement Covariance Matrix
P = diag([0.1, 0.1, 0.1, 10, 10, 10, 5, 5, 5]);     % State Covariance Matrix
                              
H = [eye(3), zeros(3,3), zeros(3,3)];               % H Matrix

% Initializing paramters for Q (Process Covariance Matrix)
qx = 50;    
qy = 75;
qz = 100;

% Initializing State Vector Variables
px = x_ned(t_min_ind);
py = y_ned(t_min_ind);
pz = z_ned(t_min_ind);

vx = 0.0;
vy = 0.0;
vz = 0.0;

bx = 0;
by = 0;
bz = 0;


% Biases can also be initialized by performing sensor calibration

%t_int = t_min + 300;
%t_int_ind = find(t >= t_int, 1);
%bx = mean(acx(t_min_ind:t_int_ind));
%by = mean(acy(t_min_ind:t_int_ind));
%bz = mean(acz(t_min_ind:t_int_ind)) + 9.81;



% Initial State Vector
state = [px;py;pz;vx;vy;vz;bx;by;bz];

% Array to store state estimations
state_estimations = zeros(size(state));


for i = t_min_ind : t_max_ind

    dt = t(i+1) - t(i);
    yk = [x_ned(i+1); y_ned(i+1); z_ned(i+1)];

    % Define the Rotation Matrix
    R_gb = [cos(psi(i))*cos(tht(i)), cos(psi(i))*sin(tht(i))*sin(phi(i)) - sin(psi(i))*cos(phi(i)), cos(psi(i))*sin(tht(i))*cos(phi(i)) + sin(psi(i))*sin(phi(i));
         sin(psi(i))*cos(tht(i)), sin(psi(i))*sin(tht(i))*sin(phi(i)) + cos(psi(i))*cos(phi(i)), sin(psi(i))*sin(tht(i))*cos(phi(i)) - cos(psi(i))*sin(phi(i));
         -sin(tht(i)), cos(tht(i))*sin(phi(i)), cos(tht(i))*cos(phi(i))];

    % Defining the F matrix
    F_upper_right = [eye(3), diag([dt, dt, dt]), diag([-(dt^2/2), -(dt^2/2), -(dt^2/2)]);
                     zeros(3,3), eye(3), diag([-dt, -dt, -dt])];
    F_upper_left =  [eye(6), zeros(6,3); zeros(3,6), R_gb];
    F_upper      =  F_upper_right * F_upper_left;
    F_lower      =  [zeros(3,3), zeros(3,3), eye(3)];
    F            =  [F_upper; F_lower];

    % Defining the G Matrix
    G_left  =  [diag([(dt^2/2), (dt^2/2), (dt^2/2)]); diag([dt, dt, dt]); zeros(3,3)];
    G_right =  [R_gb, [0; 0; 9.81]];
    G       =  G_left * G_right;

    % Defining the Q covariance matrix
    Q = G * diag([qx, qy, qz, 0]) * G';


    % Prediction Step
    state_pred = F * state + G * [acx(i);acy(i);acz(i);1];
    P_pred = F * P * F' + Q;

    % Correction Step is done only after getting GPS update
    if rem(i, 5) == 0
        K = P_pred * H' * inv(H * P_pred * H' + R_cov);
        state = state_pred + K * (yk - (H * state_pred));
        P = (eye(9) - K * H) * P_pred;
    else
        state = state_pred;
        P = P_pred;
    end

    state_estimations = [state_estimations, state];

end

state_estimations = state_estimations(:,2:end);

%% Result plots
% NED Co-ordinates vs Estimated Co-ordinates
figure; set(gcf,'numbertitle','off','name','NED - Coordinates vs State Estimations');  

subplot(3,1,1);
hold on;
plot(t(t_min_ind:t_max_ind), x_ned(t_min_ind:t_max_ind), 'b'); ylabel('X'); grid on;
plot(t(t_min_ind:t_max_ind), state_estimations(1, :), 'r'); grid on;
legend('NED Co-ordinates', 'State Estimated Co-ordinates');
hold off;

subplot(3,1,2);
hold on;
plot(t(t_min_ind:t_max_ind), y_ned(t_min_ind:t_max_ind), 'b'); ylabel('Y'); grid on;
plot(t(t_min_ind:t_max_ind), state_estimations(2, :), 'r'); grid on;
legend('NED Co-ordinates', 'State Estimated Co-ordinates');
hold off;
 
subplot(3,1,3);
hold on;
plot(t(t_min_ind:t_max_ind), z_ned(t_min_ind:t_max_ind), 'b'); ylabel('Z'); grid on; xlabel('time (s)');
plot(t(t_min_ind:t_max_ind), state_estimations(3, :), 'r'); grid on;
legend('NED Co-ordinates', 'State Estimated Co-ordinates');
hold off;

%% Acceleration Bias Plots

figure; set(gcf,'numbertitle','off','name','Acceleration Biases');

subplot(3,1,1);
plot(t(t_min_ind:t_max_ind), state_estimations(7, :), 'b'); ylabel('bx'); grid on;

subplot(3,1,2);
plot(t(t_min_ind:t_max_ind), state_estimations(8, :), 'b'); ylabel('by'); grid on;

subplot(3,1,3);
plot(t(t_min_ind:t_max_ind), state_estimations(9, :), 'b'); ylabel('bz'); grid on;xlabel('time (s)');


%% Velocity Plots 
figure; set(gcf,'numbertitle','off','name','Estimated Velocity');

subplot(3,1,1);
plot(t(t_min_ind:t_max_ind), state_estimations(4, :), 'b'); ylabel('vx'); grid on;

subplot(3,1,2);
plot(t(t_min_ind:t_max_ind), state_estimations(5, :), 'b'); ylabel('vy'); grid on;

subplot(3,1,3);
plot(t(t_min_ind:t_max_ind), state_estimations(6, :), 'b'); ylabel('vz'); grid on;xlabel('time (s)');

