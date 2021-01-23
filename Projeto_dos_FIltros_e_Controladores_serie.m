clear
clc
close all

% Calculo dos parametros dos filtros e controladores:

%% Coeficientes do filtro de segunda ordem (CCS)

% Frequencia de amostragem:
fsample = 18000;
Ts = 1/fsample;
format long

fso = 10;                                                                  % frequência do filtro [Hz]

Cf0 = (Ts^2)*((2*pi*fso)^2);
Cf1 = abs(((Ts)*((2*pi*fso))*sqrt(2)) - ((Ts^2)*((2*pi*fso)^2)) - 1);
Cf2 = 2 - (Ts*2*pi*fso*sqrt(2));

fprintf('Coeficientes do Filtro de Segunda Ordem:\n\n')
display(num2str(Cf0,'Cf0 = %.20f'));
display(num2str(Cf1,'Cf1 = %.20f'));
display(num2str(Cf2,'Cf2 = %.20f'));

%% Coeficientes do filtro de Primeira ordem

% Frequencia de amostragem:
fsample = 18000;
Ts = 1/fsample;
format long

fso = 20;                                                                  % frequência do filtro [Hz]

Cf0 = Ts/(1/(2*pi*fso));
Cf1 = Ts/(1/(2*pi*fso));

fprintf('Coeficientes do Filtro de Primeira Ordem:\n\n')
display(num2str(Cf0,'Cf0 = %.20f'));
display(num2str(Cf1,'Cf1 = %.20f'));

%% Coeficientes controlador ressonante
clc;
clear;
Ts = 1/(18000);
wn = 2*pi*60;
h = 11;

s = tf('s');
Gc =  s/(s^2+(h*wn)^2);
opt = c2dOptions('Method','tustin','PrewarpFrequency',h*wn);
Gcz = c2d(Gc,Ts,opt);
[num,den,Tsm] = tfdata(Gcz);

c1 = num{1}(1)/den{1}(1);
c2 = num{1}(3)/den{1}(1);
c3 = den{1}(2)/den{1}(1);
c4 = den{1}(3)/den{1}(1);

display(num2str(c1,'c1 = %.20f'));
display(num2str(c2,'c2 = %.20f'));
display(num2str(c3,'c3 = %.20f'));
display(num2str(c4,'c4 = %.20f'));

%% Projeto filtro passa baixa segunda ordem
clc
clear
s = tf('s');
zeta = 0.707;
wn = 2*pi*2.5;
Ts = 1/18000;
z = tf('z',Ts);

format long
Hd = wn^2 / (s^2 + 2*zeta*wn*s + wn^2);

Hdz = c2d(Hd,Ts,'tustin');

step(Hd);

[num,den,Tsm] = tfdata(Hdz);

% y = x*c0 + x*c1*z^-1 + x*c2*z^-2 - y*c3*z^-1 - y*c4*z^-2

c0 = num{1}(1)/den{1}(1);
c1 = num{1}(2)/den{1}(1);
c2 = num{1}(3)/den{1}(1);
c3 = den{1}(2)/den{1}(1);
c4 = den{1}(3)/den{1}(1);

display(num2str(c0,'c0 = %.20f'));
display(num2str(c1,'c1 = %.20f'));
display(num2str(c2,'c2 = %.20f'));
display(num2str(c3,'c3 = %.20f'));
display(num2str(c4,'c4 = %.20f'));

%% Projeto ganho controle do Vdc quadrado
clear; close all; clc;

s = tf('s');
Cdc = (2/3)*(4.7e-3);
fc = 20;
wc = 2*pi*fc;
MF = 60;
%Código

Tplanta = (2/(Cdc*s))
%%Tplanta = (2/(Cdc*s));

[Amp,PH] = bode(Tplanta,wc);
Gain_PI = 1/Amp;
Phase_PI = -PH - 180 + MF;
Tan_PI = (1/wc)*tan(pi*(Phase_PI + 90)/180);
Kpi = (Gain_PI*wc)/(sqrt(1+(wc*Tan_PI)^2));
KP = Kpi*Tan_PI
KI = Kpi

Gc = KP+KI/s;
MA = Tplanta*Gc;

bode(MA)

%% Equalização dos módulos
clear; close all; clc;

s = tf('s');
Cdc = 4.7e-3;
fc = 4;
wc = 2*pi*fc;
MF = 60;
%Código

Tplanta = (2/(Cdc*s))
%%Tplanta = (2/(Cdc*s));

[Amp,PH] = bode(Tplanta,wc);
Gain_PI = 1/Amp;
Phase_PI = -PH - 180 + MF;
Tan_PI = (1/wc)*tan(pi*(Phase_PI + 90)/180);
Kpi = (Gain_PI*wc)/(sqrt(1+(wc*Tan_PI)^2));
KP = Kpi*Tan_PI
KI = Kpi

Gc = KP+KI/s;
MA = Tplanta*Gc;

bode(MA)

%% Projeto ganho malha de reativo
clear; close all; clc;

s = tf('s');
fc = 2;
wc = 2*pi*fc;

Ki = wc

%% Projeto ganho controle de corrente do conversor Dc/Dc 
clear;
clc;
close all;

fsw = 18000;
fc = fsw/20;
Vdc = 450/2;
R = 0.55;
L = 4e-3;
Nbseries = 16;
Nbstrins = 1;
Rint = 7.1e-3 * Nbseries/Nbstrins;

K = Vdc/(R+Rint);
Tm = L/(R+Rint);

KP = 2*pi*fc*Tm/K;
KI = 2*pi*fc/K;

disp('____________________________________________________');
disp('-------------Ganhos do Controle de Corrente do Boost----------------');
disp('____________________________________________________');
disp({'KP =',num2str(KP)});
disp({'KI =',num2str(KI)});

disp('____________________________________________________');
disp('-------------Ganhos do Controle de Corrente do Buck----------------');
disp('____________________________________________________');
disp({'KP =',num2str(KP)});
disp({'KI =',num2str(KI)});

Ke = Rint;
fc1 = 10;
fc2 = fc1/20;

KP = 2*pi*fc2/(2*pi*Ke*(fc1-fc2));
KI = 2*pi*fc1*KP;

%MF = (KP*Rint*s^2 + (KI*Rint+KP*Rint*wc)*s + KI*Rint*wc)/(s^2 + (wc+wc*KP*Rint)*s + KI*Rint*wc);
%pzmap(MF)
% Tm = 1/(2*pi*60);
% fc = 1;
% 
% KP = 2*pi*fc*Tm/Ke;
% KI = 2*pi*fc/Ke;

disp('____________________________________________________');
disp('-------------Ganhos do Controle de Tensão do Buck----------------');
disp('____________________________________________________');
disp({'KP =',num2str(KP)});
disp({'KI =',num2str(KI)});


% %% Projeto ganho controle do Vdc quadrado
% clear; close all; clc;
% 
% s = tf('s');
% Cdc = 2*(4.7e-3);
% fc1 = 10;
% fc2 = 1;
% Vd = 35*sqrt(2/3);
% Vdc = 80;
% G = -(3/2)*Vd/Vdc;
% H = -3/2 * Vd;
% 
% Kp = (2*pi/G)*(fc1+fc2)*Cdc
% Ki = ((4*pi^2)/G)*fc1*fc2*Cdc
% 
% Kpq = 2*pi*fc2/(2*pi*H*(fc1-fc2))
% Kiq = 2*pi*fc1*Kpq


