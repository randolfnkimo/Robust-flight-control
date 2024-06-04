clear
clc
close all 
%% Part #1 System modeling

% Data values 
g=9.80665;
a=316.0561;
M=3;
Za=1236.8918;
Ma=-300.4211;
Mq=0;
Zd=108.1144;
Md=-131.3944;
Aa=1434.7783;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
Ad=115.0529;
wa=150;
zeta=0.7;
V=M*a;

% Actuator's's state space model
Aac = [0 1; -wa^2 -2*zeta*wa];
Bac = [0 wa^2]';
Cac = eye(2);
Dac = zeros(2,1);
G_a = ss(Aac,Bac,Cac,Dac, 'StateName',{'x3','x4'},...
    'InputName','u_cmd','OutputName',{'u_m','udot_m'});

% Missile's state space model
Am = [-Za/V 1; Ma Mq];
Bm = [-Zd/V Md]';
Cm = [-Aa/g 0; 0 1];
Dm = [-Ad/g 0]';
G_m = ss(Am,Bm,Cm,Dm, 'StateName',{'x1','x2'},...
    'InputName','u_m','OutputName',{'y1','y2'});

% Airframe's state space model
G_am = linearize('Airframe');
G_am = ss2ss(G_am,[0 0 1 0; 0 0 0 1;1 0 0 0;0 1 0 0]);
G_am_nz = zpk(G_am(1,1));
G_am_q  = zpk(G_am(2,1));
%%  Part #2 Loop shaping
G_ol_q = G_am(2,1);
%siso(G_ol_q)                              % Tool to find precise C_q
C_q = zpk(-0.15769);
G_cl_q_unsc = linearize('ClosedLoop_Cq');

% Scaling gain
C_sc = zpk(inv(dcgain(G_cl_q_unsc)));

% Total precompensated inner loop transfer function
G = linearize('ClosedLoop_CqCsc');
zpk(G);
% Integral gain design
C_i = zpk(1);
G_ol_nz = linearize('ClosedLoop_CqCscCi');
C_i = zpk(5.9041);
T=zpk(feedback(G*C_i/tf('s'),1,-1));
%% Part #3a Weighting filters
% W1 (So)
dcgain_W1   = db2mag(-40);
freq_W1     = 4;
mag_W1      = db2mag(-3.01);
hfgain_W1   = 2; % need to be less than 1/(2*sin(30*pi/(180*2))) = 1.9319(approx 2) to have a PM >= 30 degrees

% W2 (CSo)
dcgain_W2   = hfgain_W1;
freq_W2     = bandwidth(G_a(1,1),-3.01);
mag_W2      = db2mag(-15);
hfgain_W2   = dcgain_W1;

W1  = inv(makeweight(dcgain_W1,[freq_W1,mag_W1],hfgain_W1));
W2  = inv(makeweight(dcgain_W2,[freq_W2,mag_W2],hfgain_W2));

save W1 W1
save W2 W2

figure
sigma(inv(W1),'r-',inv(W2),'b-')
grid on;xlim([1e-3 1e4])
legend('inv W1', 'inv W2')


% computation of Ai, wi, and Mi form previous data
A1 = dcgain_W1
M1 = hfgain_W1
w1 = sqrt(((1/mag_W1^2)*freq_W1^2 - (freq_W1^2)/hfgain_W1^2)/(1 - (1/mag_W1^2) * dcgain_W1^2))

A2 = dcgain_W2
M2 = hfgain_W2
w2 = sqrt(((1/mag_W2^2)*(freq_W2*hfgain_W2)^2 - freq_W2^2)/(1/dcgain_W2^2 - (1/mag_W2^2)))

%% Part #3b Reference model
zm=36.64;
acc_chan = tf([-1,zm],zm);

% Desired parameters
desired_overshoot = 0.05;
desired_response_time = 0.18;

% Initial values of wn and zeta
zeta_i = -log(desired_overshoot)/sqrt(pi^2+log(desired_overshoot)^2);
wd_i = 3.5/desired_response_time;
% Call fmincon to adjust parameters
options = optimoptions('fmincon','StepTolerance',1e-15);
x0 = [wd_i, zeta_i];
lb = [0, 0]; 
ub = [inf, 1]; 
[opt_params, fval] = fmincon(@(x) objective_function(x, desired_overshoot, desired_response_time,acc_chan), x0, [], [], [], [], lb, ub, [], options);

% Optimal parameters
wn_opt = opt_params(1);
zeta_opt = opt_params(2);
fprintf('Paramètres optimaux :\n');
fprintf('wn_n = %.4f\n', wn_opt);
fprintf('zeta = %.4f\n', zeta_opt);

% Display minimized error and calculate T_d
fprintf('Erreur minimisée = %.6f\n', fval);
G_opt = tf(wn_opt^2, [1, 2*zeta_opt*wn_opt, wn_opt^2]);
T_d = series(G_opt,acc_chan);
%step(T_d)
%% Part #3c Feedback controller design (hinfsyn case)
%W3=W1;
dcgain_W3   = db2mag(-40);
freq_W3     = 4;
mag_W3      = db2mag(-20);
hfgain_W3   = 2;
W3  = inv(makeweight(dcgain_W3,[freq_W3,mag_W3],hfgain_W3));

P = linearize('Design');
opts = hinfsynOptions('RelTol',1e-6);
[C0_e,T_wz,gamma] = hinfsyn(P,1,1,opts);

opt = sigmaoptions;
opt.MagUnits = 'abs';

figure
sigma(T_wz,T_wz(1,1),T_wz(2,1),T_wz(3,1),opt)
legend('T_wz(s)','T_wz1(s)','T_wz2(s)','T_wz3(s)')
grid on

norm_inf_T_wz = norm(T_wz,'inf');
gamma_1 =  norm(T_wz(1,1),'inf');
gamma_2 =  norm(T_wz(2,1),'inf');
gamma_3 =  norm(T_wz(3,1),'inf');

% figure(2)
% sigma(W1\T_wz(1,1),W2\T_wz(2,1),W3\T_wz(3,1))
% legend('So','C_e*So','T_d-T_o')

[z_C0_e,p_C0_e,k_C0_e] = zpkdata(C0_e,'v');

C_e_min = zpk(z_C0_e([2 3 4 5 6 7]),p_C0_e([2 3 4 5 6 7 9]),k_C0_e * abs(z_C0_e(1)) * abs(z_C0_e(8)) / (abs(p_C0_e(1)) * abs(p_C0_e(8))));

C_i_min = zpk(z_C0_e([2 3 4 5 6 7]),p_C0_e([2 3 4 5 6 7]),k_C0_e * abs(z_C0_e(1)) * abs(z_C0_e(8)) / (abs(p_C0_e(1)) * abs(p_C0_e(8))));

C_i_red = balred(C_i_min,2);

C_i = C_i_red;
figure
bode(C0_e,C_e_min)
legend('C0_e','C_e_min')
grid on
figure
bode(C_i_min,C_i_red)
legend('C_i_min','C_i_red')
grid on
figure
iopzmap(C_i_min,C_i_red)
grid on

% Feedfoward controller
F_f = zpk(1);

% Closed loop transfert function
T = linearize('ClosedLoop_Test');
So = T(1,1);
CeSo = T(2,1);
To = T(3,1);
Tm = T(4,1);
Ti = - T(2,2);
SoG = T(3,2);
Si = T(5,2);

% 2x3 plots
subplot(2,3,1)
sigma(inv(W1),'r-',So,'b-',Si,'b-')
grid on

subplot(2,3,2)
sigma(inv(W2),'r-',CeSo,'b-')
grid on

subplot(2,3,3)
sigma(inv(W3),'r-',Tm,'b-')
grid on

subplot(2,3,4)
sigma(Ti,'b-',To,'b-')
grid on

subplot(2,3,5)
sigma(SoG,'b-')
grid on

subplot(2,3,6)
C_e_red = C_i_red*tf(1,[1 0]);
sigma(C0_e,'b-',C_e_red,'b-')
grid on

OpenLoop = linearize('OpenLoop_Test');
figure
margin(OpenLoop)

figure

subplot(2,2,1)
step(So,'b-')
grid on

subplot(2,2,2)
step(T_d,'r-',To,'b-')
grid on

subplot(2,2,3)
step(SoG,'b-')
grid on

subplot(2,2,4)
step(T(6,1)*180/pi,'b-')
grid on

%% Part #3d Feedback controller desing (hinfstruct case)
Np = 2;
Nz = 2;
C_i_red2_init = tunableTF('C_e_red2_init',Nz,Np);
C_e_red2_init = C_i_red2_init*tf(1,[1 0]);

% parpool
options = hinfstructOptions('RandomStart',20,'UseParallel',true,'TolGain',1e-6);
[C_e_red2,gamma,tuning_info] = hinfstruct(P,C_e_red2_init,options);
[z_C_e_red,p_C_e_red,k_C_e_red] = zpkdata(C_e_red2,'v');
C_i_red2 = zpk(z_C_e_red([1 2]),p_C_e_red([1 2]),k_C_e_red);

T_wz2 = lft(P,C_e_red2,1,1);

opt = sigmaoptions;
opt.MagUnits = 'abs';
figure;
sigmaplot(T_wz2,T_wz2(1,1),T_wz2(2,1),T_wz2(3,1),opt);
legend('sigma of T_wz2','sigma of T_wz2_1','sigma of T_wz2_2','sigma of T_wz2_3');

figure;
bode(C_i_min,'b',C_i_red,'g',C_i_red2,'m');


C_i = C_i_red2;
F_f = 1;
T2 = tf(linearize('ClosedLoop_Test'));


So2 = T2(1,1);
CeSo2 = T2(2,1);
To2 = T2(3,1);
Tm2 = T2(4,1);
Ti2 = - T2(2,2);
SoG2 = T2(3,2);
Si2 = T2(5,2);

figure
subplot(2,3,1)
sigma(inv(W1),'r',So,'b-',Si,'b-',So2,'m-',Si2,'m-')
grid on

subplot(2,3,2)
sigma(inv(W2),'r',CeSo,'b-',CeSo2,'m-')
grid on

subplot(2,3,3)
sigma(inv(W3),'r',Tm,'b-',Tm2,'m-')
grid on

subplot(2,3,4)
sigma(Ti,'b-',To,'b-',Ti2,'m-',To2,'m-')
grid on

subplot(2,3,5)
sigma(SoG,'b-',SoG2,'m-')
grid on

subplot(2,3,6)
sigma(C0_e,'r-',C_e_red,'b-',C_e_red2,'m-')
grid on

OpenLoop2 = linearize('OpenLoop_Test');
figure
margin(OpenLoop2)

figure

subplot(2,2,1)
step(T(1,1),'b-',T2(1,1),'m-')
grid on

subplot(2,2,2)
step(T_d,'r-',T(3,1),'b-',T2(3,1),'m-')
grid on

subplot(2,2,3)
step(T(3,2),'b-',T2(3,2),'m-')
grid on

subplot(2,2,4)
step(T(6,1)*180/pi,'b-',T2(6,1)*180/pi,'m-')
grid on

%% Part #3e Feedforword controller design
T0 = T2(3,1);                   % Output Sensivity Function with new Values
F_f_init = T_d * inv(T0);       % First Feedback controller
zpk(F_f_init);                  % Output
% figure;
% sigma(F_f_init,'b-');           % Singular value plot

% % Controller order reduction
% figure;
% iopzmap(F_f_init);              % Plot of zeros and poles                                                  
% grid on;

% Truncated controller F_f_lf
[z_F_f_init,p_F_f_init,k_F_f_init] = zpkdata(F_f_init,'v');
F_f_lf = zpk(z_F_f_init([4 5 6 7 8]),p_F_f_init([2 3 4 5 6]), (k_F_f_init / abs(p_F_f_init(1))) * abs(z_F_f_init([1]))^2 * abs(z_F_f_init([3])) );
% Comparaison of the dc gains
dcgain(F_f_lf);     % DC Gain of F_f_lf
dcgain(F_f_init);   % DC Gain of F_f_init
zpk(F_f_lf);        % Output of F_f_lf

% % Comparaison of singular values
% figure;
% sigma(F_f_init,'b-',F_f_lf,'r-');
% grid on;

% Model order reduction techniques
balred_opts = balredOptions('ErrorBound','absolute');
F_f = balred(F_f_lf,3,balred_opts);                         % Reduced order controller
zpk(F_f);

% % Comparaison of the pole zero map 
% figure;
% iopzmap(F_f,'b-',F_f_init,'r-');
% grid on;

% Controller analysis & simulation
C_i = C_i_red2;                         % Upload optimized model to Simulink
T3 = linearize('ClosedLoop_Test');      % Linearization

% % Comparaison of singular values
% figure;
% sigma(inv(W3),'r-',T(4,1),'b-',T2(4,1),'m-',T3(4,1),'g-');
% grid on

% % Comparaison of controllers
% figure;
% sigma(C_i_min,'r-',C_i_red,'b-',C_i_red2,'m-',F_f,'g-');
% grid on;

% % Comparison of the output sensitivity function 
% figure;
% step(T_d,'r-',T(3,1),'b-',T2(3,1),'m-',T3(3,1),'g-');
% grid on;

% % Comparison of the actuator deflection rate
% figure;
% step(T(6,1)*180/pi,'b-',T2(6,1)*180/pi,'m-',T3(6,1)*180/pi,'g-');
% grid on;

%%
% % Last comparison
figure;
subplot(2,2,1);
sigma(inv(W3),'r-',T(4,1),'b-',T2(4,1),'m-',T3(4,1),'g-');
grid on;
subplot(2,2,2);
step(T_d,'r-',T(3,1),'b-',T2(3,1),'m-',T3(3,1),'g-');
grid on;
subplot(2,2,3);
sigma(C_i_min,'r-',C_i_red,'b-',C_i_red2,'m-',F_f,'g-');
grid on;
subplot(2,2,4);
step(T(6,1)*180/pi,'b-',T2(6,1)*180/pi,'m-',T3(6,1)*180/pi,'g-');
grid on;


%% Part #4 Feedback controller redesign (systune case)
F_f = 1;
%% fmincon objective function for Reference model
function error = objective_function(x, desired_overshoot, desired_response_time,acc_chan)
    wn = x(1);
    zeta = x(2);

    % Définir la fonction de transfert en boucle ouverte avec les paramètres donnés
    G = tf(wn^2, [1, 2*zeta*wn, wn^2]);
    G = series(G,acc_chan);
    % Obtenir les informations 
    info = stepinfo(G,'SettlingTimeThreshold',0.05);
    % Calcul du dépassement
    overshoot = (info.Overshoot)/100;

    % Calcul du temps de réponse à 5%
    response_time = info.SettlingTime;

    % Calcul de l'erreur (somme des carrés des écarts)
    error = (overshoot - desired_overshoot)^2 + (response_time - desired_response_time)^2;
end