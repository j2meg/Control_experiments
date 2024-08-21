%                    A
% H(s) = ———————————————————————————
%        1 + ps1 s + ps2 s² + ps3 s³
clear all
close all

%% Filter parameters
R1=1e+3;
R2=1e+3 ;
R3=2e+3;
R4=100 ;
C1=2.2e-9;
C2=15e-9;
C3=470e-12;
%% gain
A=-R3/(R1+R2);
%% Parameters
ps1= (C1*R1*R2+C3*(R3*R4+(R1+R2)*(R3+R4)))/(R1+R2);
ps2= (C3*(C1*R1*(R3*R4+R2*R3+R2*R4)+C2*R3*R4*(R1+R2)))/(R1+R2);
ps3= (C1*C2*C3*R1*R2*R3*R4)/(R1+R2);

%%
k_1=1;
k_2=1;
q=1;
fc=145467.6179859923;
ps1_w = 1/(k_1*2*pi*fc)+1/(q*k_2*2*pi*fc);
ps2_w = 1/(k_1+2*pi*fc*q*k_2*2*pi*fc)+1/((k_2*2*pi*fc)^2);
ps3_w = (C1*C2*C3*R1*R2*R3*R4)/(R1+R2);
%%
fprintf('Polos dados por los componentes:\n')
fprintf('ps1=%e, ps2=%e, ps3=%e\n',ps1,ps2,ps3)
fprintf('Polos dados por parametros de diseño:\n')
fprintf('ps1=%e, ps2=%e, ps3=%e\n',ps1_w,ps2_w,ps3_w)
%% Transfer functions
H=tf([A],[ps3 ps2 ps1 1])
H_w=tf([A],[ps3_w, ps2_w, ps1_w, 1])
%% roots
H_roots=roots([ps3 ps2 ps1 1])
figure(1)
plot(real(H_roots),imag(H_roots),"o")
axis equal
grid on
xlabel("Re(z)")
ylabel("Im(z)")
title('Raices H')
xline(0)
yline(0)

H_w_roots=roots([ps3_w ps2_w ps1_w 1])
figure(2)
plot(real(H_w_roots),imag(H_w_roots),"o")
axis equal
grid on
xlabel("Re(z)")
ylabel("Im(z)")
title("Raices H_w")
xline(0)
yline(0)


%% Bode diagrams
figure(3)
bode(H,'red')
title('Bode planta por componentes')
figure(4)
bode(H_w,'green')
title('Bode planta por diseño')
%xline(0)
yline(-3)

%%Frecuencias de corte de radianes/s a hertz
FC=9.18e+05/(2*pi)
FC_w=9.49e+05/(2*pi)
figure(5)
step(H)
title('Respuesta transitoria H')
figure(6)
step(H_w)
title('Respuesta transitoria H_w')
%xline(0)
yline(-3)
