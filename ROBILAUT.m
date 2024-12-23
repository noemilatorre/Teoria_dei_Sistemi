%esame
clear all
clc
%% INIZIALIZZAZIONE DATI: 
%dimensioni mat A 
n=3;  
%condizioni stato iniziali
x=0;
y=0;
theta=0;

delta=0.1;
alpha=0.2;                        

%% INIZIALIZZAZIONE FILTRO DI KALMAN

Ps = 100*diag([0.01 0.01 1]);     %(3x3)
P = 1*eye(n,n);                   % Inizializzazione della covarianza 
%R_w > R_v = (buone misure->alto guadagno)  
R_w = 1*diag([0.1 0.1 0.5]);      % R_w - covarianza del rumore di processo
R_v = 0.5*diag([0.1 0.1 1]);      % R_v - covarianza del rumore di misura

%% PARAMETRI PER MPC 

% coordinate desiderate
xd = 10;   
yd = 2;
theta_d = atan2(yd - y, xd - x); 

%vincoli sull'ingresso
u_min=[-0.5; -0.3];
u_max=[0.5; 0.3];

% u_min=[-0.8; -0.2];
% u_max=[0.8; 0.2];


%% SCELTA DEL CONTROLLORE DA UTILIZZARE: 

 N = 101;    %lqr 
% N = 151;    %mpc_non_vincolato
% N = 301;    %mpc_vincolato

k = 0:N-1;
S0.u=zeros(2,N);
S1.u=zeros(2,N);
S2.u=zeros(2,N);

  d=0;   %esegue lqr
 % d=1;   %esegue mpc NON VINCOLATO
 % d=2;   %esegue mpc VINCOLATO

%% IMPLEMENTAZIONE LQR

if (d==0)
S0.x_hats(:,1) = [1; 2; 3];   
S0.x(:,1) = [0,0,0]; 

S0.e(1,1)=(xd-S0.x_hats(1,1));
S0.e(2,1)=(yd-S0.x_hats(2,1));
S0.e(3,1)=(theta_d-S0.x_hats(3,1));

% parametri ottimizzazione: J = sum_N (x'Qx + u'Su )
% Matrice di peso di Stato: serve per assegnare a degli
% stati in particolare, più peso rispetto agli altri

Q = 10*diag([0.01 0.01 1]);   %Matrice di peso dell'errore nel transitorio (nxn)
S = 1*diag([1 1.2]);          %Matrice di pesp dello sforzo di controllo (pxp) 

for i = 1:N 
%modifico la v alla prima iterazione del ciclo for, per rendere la coppia (A,B) stabilizzabile
    v = S0.u(1,i) + 0.000001; 
    omega = S0.u(2,i);
%orientamento di theta_d varia ad ogni iterazione
    theta_d(i) = atan2((yd - S0.x_hats(2,i)), (xd - S0.x_hats(1,i)));              
  
A = [0 0 -delta*omega*cos(S0.x_hats(3,i))-v*sin(S0.x_hats(3,i));
     0 0 -delta*omega*sin(S0.x_hats(3,i))+v*cos(S0.x_hats(3,i));
     0 0 0];

B = [cos(S0.x_hats(3,i)) -delta*sin(S0.x_hats(3,i));
     sin(S0.x_hats(3,i)) +delta*cos(S0.x_hats(3,i));
     0 1];

C = [1 0 (alpha+delta)*sin(S0.x_hats(3,i));
     0 1 (alpha-delta)*cos(S0.x_hats(3,i));
     0 0 1];

% Discretizzazione usando il metodo di Eulero 
        T = 0.1; % Passo di campionamento
        A = eye(size(A)) + A*T;
        B = B*T; 
      
% Matrici nel modello dell'errore:
%  A=A
%  B=-B
%  C=mat identità (3x3). Matrice in cui evidenzio i sensori (x,y,theta)

  S0.e(:,i) = [xd-S0.x_hats(1,i); 
               yd-S0.x_hats(2,i);
           theta_d(i)-S0.x_hats(3,i)];

%Chiamo funzione mympc per LQR: u = my_mpc(A,B,C,x0,Q,S)
S0.u(:,i) = my_mpc(A,-B,eye(3,3),S0.e(:,i),Q,S);  

% Simulazione del sistema dinamico
    if (i < N)
        S0.x(1,i+1) = S0.x(1,i) + T*(S0.u(1,i)*cos(S0.x(3,i))-delta*S0.u(2,i)*sin(S0.x(3,i)));
        S0.x(2,i+1) = S0.x(2,i) + T*(S0.u(1,i)*sin(S0.x(3,i))+delta*S0.u(2,i)*cos(S0.x(3,i)));
        S0.x(3,i+1) = S0.x(3,i) + T*(S0.u(2,i));  
    end
        S0.y(1,i) = S0.x(1,i)-delta*cos(S0.x(3,i))-alpha*cos(S0.x(3,i));
        S0.y(2,i) = S0.x(2,i)-delta*sin(S0.x(3,i))+alpha*sin(S0.x(3,i));
        S0.y(3,i) = S0.x(3,i);
           
  % calcolo del guadagno
  K = Ps*C'*inv(R_v + C*Ps*C');
  % equazioni di aggiornamento di misura (non lineare)
  
  S0.x_hat(:,i) = S0.x_hats(:,i) + K*(S0.y(:,i) + [-S0.x_hats(1,i)+delta*cos(S0.x_hats(3,i))+alpha*cos(S0.x_hats(3,i));
                                                   -S0.x_hats(2,i)+delta*sin(S0.x_hats(3,i))-alpha*sin(S0.x_hats(3,i));
                                                   -S0.x_hats(3,i)]);
  P = Ps - K*C*Ps;

  % equazioni di aggiornamento temporale (non lineare)
  if (i<N)
    S0.x_hats(:,i+1) = S0.x_hat(:,i) + T*([S0.u(1,i)*cos(S0.x_hat(3,i))-delta*S0.u(2,i)*sin(S0.x_hat(3,i)); 
                                          S0.u(1,i)*sin(S0.x_hat(3,i))+delta*S0.u(2,i)*cos(S0.x_hat(3,i)); 
                                          S0.u(2,i)]); 
  
    Ps = A*P*A' + R_w;
 
  end

end

%plot MPC
subplot(4,1,1)
plot(k,S0.u)
hold on 
legend('lqr')
grid on
title('u(k)');

subplot(4,1,2)
plot(k,S0.x_hat)
grid on
title('xhat');

subplot(4,1,3)
plot(k,(S0.x - S0.x_hat))
grid on 
title('errore');

subplot(4,1,4)
plot(k,S0.y)
grid on
title('uscita');
legend('x', 'y', '\theta')

end

%% CASO MPC NON VINCOLATO 

if (d==1)
 
% S1.x_hats(:,1) = [1; 2; 3];  
S1.x_hats(:,1) = [10; 2; 0];  
S1.x(:,1)=[0,0,0];

S1.e(1,1)=(xd-S1.x_hats(1,1));
S1.e(2,1)=(yd-S1.x_hats(2,1));
S1.e(3,1)=(theta_d-S1.x_hats(3,1));

% parametri ottimizzazione: J = sum_N (x'Qx + u'Su ) + x(N)'Vx(N) 

Q = 10*diag([0.01 0.01 0.2]);   
S = 1*diag([1 1.2]);  
V = 10*diag([0.01 0.01 0.2]); 

for i = 1:N 

    v=S1.u(1,i);
omega=S1.u(2,i);
    theta_d(i) = atan2((yd - S1.x_hats(2,i)), (xd - S1.x_hats(1,i)));         

A = [0 0 -delta*omega*cos(S1.x_hats(3,i))-v*sin(S1.x_hats(3,i));
     0 0 -delta*omega*sin(S1.x_hats(3,i))+v*cos(S1.x_hats(3,i));
     0 0 0];

B = [cos(S1.x_hats(3,i)) -delta*sin(S1.x_hats(3,i));
     sin(S1.x_hats(3,i)) +delta*cos(S1.x_hats(3,i));
     0 1];

C = [1 0 (alpha+delta)*sin(S1.x_hats(3,i));
     0 1 (alpha-delta)*cos(S1.x_hats(3,i));
     0 0 1];

% Discretizzazione usando il metodo di Eulero 
        T = 0.1; % Passo di campionamento
        A = eye(size(A)) + A*T;
        B = B*T; 

   S1.e(:,i) = [xd-S1.x_hats(1,i); 
                yd-S1.x_hats(2,i);
           theta_d(i)-S1.x_hats(3,i)];


% Chiamo la funzione my_mpc per unconstrained case:u = my_mpc(A,B,C,x0,Q,S,N,P)
 S1.u(:,i) = my_mpc(A,-B,eye(3,3),S1.e(:,i),Q,S,N,V);    
% Simulazione del sistema dinamico
    if (i < N)

        S1.x(1,i+1) = S1.x(1,i) + T*(S1.u(1,i)*cos(S1.x(3,i))-delta*S1.u(2,i)*sin(S1.x(3,i)));
        S1.x(2,i+1) = S1.x(2,i) + T*(S1.u(1,i)*sin(S1.x(3,i))+delta*S1.u(2,i)*cos(S1.x(3,i)));
        S1.x(3,i+1) = S1.x(3,i) + T*(S1.u(2,i));
    end

        S1.y(1,i) = S1.x(1,i)-delta*cos(S1.x(3,i))-alpha*cos(S1.x(3,i));
        S1.y(2,i) = S1.x(2,i)-delta*sin(S1.x(3,i))+alpha*sin(S1.x(3,i));
        S1.y(3,i) = S1.x(3,i);
          
  % calcolo del guadagno
  K = Ps*C'*inv(R_v + C*Ps*C');
  % equazioni di aggiornamento di misura (non lineare)

  S1.x_hat(:,i) = S1.x_hats(:,i) + K*(S1.y(:,i) + [-S1.x_hats(1,i)+delta*cos(S1.x_hats(3,i))+alpha*cos(S1.x_hats(3,i));
                                                   -S1.x_hats(2,i)+delta*sin(S1.x_hats(3,i))-alpha*sin(S1.x_hats(3,i));
                                                   -S1.x_hats(3,i)]); 

  P = Ps - K*C*Ps;

  % equazioni di aggiornamento temporale (non lineare)
  if (i<N)

    S1.x_hats(:,i+1) = S1.x_hat(:,i) + T*([S1.u(1,i)*cos(S1.x_hat(3,i))-delta*S1.u(2,i)*sin(S1.x_hat(3,i)); 
                                           S1.u(1,i)*sin(S1.x_hat(3,i))+delta*S1.u(2,i)*cos(S1.x_hat(3,i)); 
                                           S1.u(2,i)]); 
    Ps = A*P*A' + R_w;
  end

  end

%plot MPC
subplot(4,1,1)
plot(k,S1.u)
legend('mpc non vincolato')
grid on
title('u(k)');

subplot(4,1,2)
plot(k,S1.x_hat)
grid on
title('xhat');

subplot(4,1,3)
plot(k,(S1.x - S1.x_hat))
grid on 
title('errore');

subplot(4,1,4)
plot(k,S1.y)
grid on
title('uscita');
legend('x', 'y', '\theta')
end
%% MPC  VINCOLATO

if(d==2)

S2.x_hats(:,1) = [1; 2; 3];  
S2.x(:,1) = [5,1,0];

S2.e(1,1)=(xd-S2.x_hats(1,1));
S2.e(2,1)=(yd-S2.x_hats(2,1));
S2.e(3,1)=(theta_d-S2.x_hats(3,1));

% parametri ottimizzazione: J = sum_N (x'Qx + u'Su ) + x(N)'Vx(N) 
% con u_max < u <= u_max

Q = 10*diag([0.01 0.01 0.2]);
S = 0.001*diag([1 1.2]);   
V = 10*diag([0.1 0.1 0.02]);

for i = 1:N 
    v = S2.u(1,i);
omega = S2.u(2,i);
    theta_d(i) = atan2((yd - S2.x_hats(2,i)), (xd - S2.x_hats(1,i)));
                   
A = [0 0 -delta*omega*cos(S2.x_hats(3,i))-v*sin(S2.x_hats(3,i));
     0 0 -delta*omega*sin(S2.x_hats(3,i))+v*cos(S2.x_hats(3,i));
     0 0 0];

B = [cos(S2.x_hats(3,i)) -delta*sin(S2.x_hats(3,i));
     sin(S2.x_hats(3,i)) +delta*cos(S2.x_hats(3,i));
     0 1];

C = [1 0 (alpha+delta)*sin(S2.x_hats(3,i));
     0 1 (alpha-delta)*cos(S2.x_hats(3,i));
     0 0 1];

% Discretizzazione usando il metodo di Eulero 
        T = 0.1; 
        A = eye(size(A)) + A*T;
        B = B*T;

  S2.e(:,i) = [xd-S2.x_hats(1,i); 
               yd-S2.x_hats(2,i);
           theta_d(i)-S2.x_hats(3,i)];

% Chiamo la funzione my_mpc u constrained case:u = my_mpc(A,B,C,x0,Q,S,N,P,u_min,u_max)  
  S2.u(:,i) = my_mpc(A,-B,eye(3,3),S2.e(:,i),Q,S,N,V, u_min, u_max);
     
% Simulazione del sistema dinamico
    if (i < N)

        S2.x(1,i+1) = S2.x(1,i) + T*(S2.u(1,i)*cos(S2.x(3,i))-delta*S2.u(2,i)*sin(S2.x(3,i)));
        S2.x(2,i+1) = S2.x(2,i) + T*(S2.u(1,i)*sin(S2.x(3,i))+delta*S2.u(2,i)*cos(S2.x(3,i)));
        S2.x(3,i+1) = S2.x(3,i) + T*(S2.u(2,i));  
    end
        S2.y(1,i) = S2.x(1,i)-delta*cos(S2.x(3,i))-alpha*cos(S2.x(3,i));
        S2.y(2,i) = S2.x(2,i)-delta*sin(S2.x(3,i))+alpha*sin(S2.x(3,i));
        S2.y(3,i) = S2.x(3,i);
        
  % calcolo del guadagno
  K = Ps*C'*inv(R_v + C*Ps*C');
  % equazioni di aggiornamento di misura (non lineare)
  
  S2.x_hat(:,i) = S2.x_hats(:,i) + K*(S2.y(:,i) + [-S2.x_hats(1,i)+delta*cos(S2.x_hats(3,i))+alpha*cos(S2.x_hats(3,i));
                                                   -S2.x_hats(2,i)+delta*sin(S2.x_hats(3,i))-alpha*sin(S2.x_hats(3,i));
                                                   -S2.x_hats(3,i)]);
  P = Ps - K*C*Ps;

  % equazioni di aggiornamento temporale (non lineare)
  if (i<N)
    S2.x_hats(:,i+1) = S2.x_hat(:,i) + T*([S2.u(1,i)*cos(S2.x_hat(3,i))-delta*S2.u(2,i)*sin(S2.x_hat(3,i)); 
                                           S2.u(1,i)*sin(S2.x_hat(3,i))+delta*S2.u(2,i)*cos(S2.x_hat(3,i)); 
                                           S2.u(2,i)]); 
    Ps = A*P*A' + R_w;
 
  end

  Phist(:,i) = svd(P);
  Khist(:,i) = min(svd(K));

end

%plot MPC
subplot(4,1,1)
plot(k,S2.u)
grid on
title('u(k) mpc vincolato');
legend('v','omega')

subplot(4,1,2)
plot(k,S2.x_hat)
grid on
title('xhat');

subplot(4,1,3)
plot(k,(S2.x - S2.x_hat))
grid on 
title('errore');

subplot(4,1,4)
plot(k,S2.y)
grid on
title('uscita');
legend('x', 'y', '\theta')

end
