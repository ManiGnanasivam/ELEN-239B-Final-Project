% System Constants
m = 1; %kg
b = .05; %Ns/m

% SS Model - Mass cart subject to drag forces
A = [-b/m];
B = [1/m];
C = [1];
D = [0];
sys = ss(A,B,C,D);

% Figure out PID gains using MATLAB’s built in tuner
% pidTuner(sys,'PID') % Gives: Kp=0.295 Ki=0.015 Kd=0

% Implement the PID controller into a feedback loop
Kp=0.295;
Ki=0.015;
Kd=0;
C=pid(Kp,Ki,Kd);
T = feedback(C*sys,1);

% Step response
figure(1)
subplot(2,1,1)
step(T),title('Step response with PID feedback')
subplot(2,1,2)
step(sys),title('Step response of uncontrolled system')

% Impulse response
figure(2)
subplot(2,1,1)
impulse(T),title('Impulse response with PID feedback')
subplot(2,1,2)
impulse(sys),title('Impulse response of uncontrolled system')