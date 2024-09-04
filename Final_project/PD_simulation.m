%% 2 x RC circuit proportional control
% discrete time model from https://en.wikipedia.org/wiki/Low-pass_filter#Discrete-time_realization

R1=15e3;
C1=100e-9;
R2=15e3;
C2=100e-9;
dt=700e-6; % sample time 
alpha=dt/(R1*C1+dt); % smoothing factor
t=0:dt:1;          % Simulation time
y1=zeros(1,length(t)); % 1st stage out
u1=zeros(1,length(t)); % 1st Input signal
y2=zeros(1,length(t)); % 2nd stage out
u2=zeros(1,length(t)); % 2nd stage input
setpoint=[0 1 2 3 0]; Kp=0.5; 
m=1;

for k=2:length(t)
    if mod(k,500)==0 % Change setpoint each 500 samples
        m=m+1;
    end
    error=(setpoint(m)-y2(k-1)); 
    u1(k)=u1(k-1)+Kp*error; % Proportional Control
    % saturation
    if u1(k)>3.3
        u1(k)=3.3;
    end
    if u1(k)<0
        u1(k)=0;
    end
    %% DAC
    %u(k)=u(k)*4096/3.3;
    %% process
    y1(k)=alpha*u1(k)+(1-alpha)*y1(k-1);
    y2(k)=alpha*y1(k)+(1-alpha)*y2(k-1); % system output //
                                         % In this case
                                         % u2==y1 
                                         % alpha for y1 == alpha for y2
    % saturation
    if y2(k)>3.3
        y2(k)=3.3;
    end
    if y2(k)<0
        y2(k)=0;
    end
end

subplot(1,2,1)
plot(t,y2,"Color",'r')
title("Respuesta del sistema")
subplot(1,2,2)
plot(t,u1,"Color",'b')
title("Esfuerzo de control")

