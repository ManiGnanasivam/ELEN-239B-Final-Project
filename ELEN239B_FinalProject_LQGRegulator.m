% System Constants
m = 1;
b = .05;

% SS Model - Mass cart subject to drag forces
A = [-b/m];
B = [1/m];
C = [1];
D = [0];
sys = ss(A,B,C,D);

Q = [1]; % Weighting Matrix
R = [10]; % Noise Covariance for the LQR

LQR = lqr(sys,Q,R); % Compute LQR Gain u = -K*x

% Create Covariance Matrices for disturbance (w) and noise (v)
Rw = [0.1];
Rv = [0.1];

%Create Kalman Filter
P = sys(:,[1 1]);
Kf = kalman(P,Rw,Rv); % Kalman Filter in SS Form

% Combine LQR Gain and Kalman Filter to create LQG Regulator
LQG = lqgreg(Kf,LQR);

% Connect LQG Regulator to system plant
G1 = feedback(sys,LQG,1);

% Subject system to Inputs

% Initial Condition Response
t = 0 : 0.1 : 100;
u = 0.1*randn(size(t))';
[y, t, x] = lsim(G1,u,t);
[y1, t1, x1] = lsim(sys,u,t);
figure(1)
clf
noisy = x(:,1) + Rv*randn(size(x(:,1))); %Generate noisy velocity data for comparison
plot(t,noisy,'b','Linewidth',2)
hold on;
plot(t,x(:,1),'k','Linewidth',2)
hold on;
plot(t,x(:,2),'g','Linewidth',2)
hold on;
plot(t1,x1(:,1),'r','Linewidth',2)
xlabel('Time, t [s]')
ylabel('Velocity, v [m/s]')
legend('Sensor Value','True Value','Kalman Estimate','Unregulated Velocity')
mseRand = sqrt(mean((x(:,2)-x(:,1)).^2))
rmsReg = sqrt(mean(x(:,1).^2))
rmsUnReg = sqrt(mean(x1(:,1).^2))


% Impulse Response
[y, t, x] = impulse(G1,20);
[y1, t1, x1] = impulse(sys,20);
figure(2)
clf
noisy = x(:,1) + Rv*randn(size(x(:,1))); %Generate noisy velocity data for comparison
plot(t,noisy,'b','Linewidth',2)
hold on;
plot(t,x(:,1),'k','Linewidth',2)
hold on;
plot(t,x(:,2),'g','Linewidth',2)
hold on;
plot(t1,x1(:,1),'r','Linewidth',2)
xlabel('Time, t [s]')
ylabel('Velocity, v [m/s]')
legend('Sensor Value','True Value','Kalman Estimate','Unregulated Velocity')
mseImpulse = sqrt(mean((x(:,2)-x(:,1)).^2))

% Step Response
[y, t, x] = step(G1,20);
[y1, t1, x1] = step(sys,20);
figure(3)
clf
noisy = x(:,1) + Rv*randn(size(x(:,1))); %Generate noisy velocity data for comparison
plot(t,noisy,'b','Linewidth',2)
hold on;
plot(t,x(:,1),'k','Linewidth',2)
hold on;
plot(t,x(:,2),'g','Linewidth',2)
hold on;
plot(t1,x1(:,1),'r','Linewidth',2)
hold on;
xlabel('Time, t [s]')
ylabel('Velocity, v [m/s]')
legend('Sensor Value','True Value','Kalman Estimate','Unregulated Velocity')
mseStep = sqrt(mean((x(:,2)-x(:,1)).^2))

