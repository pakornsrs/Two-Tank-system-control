%% Close loop control

clear all
clc

% Initial hight of the tank

h01_openloop = -20.1685;
h02_openloop =  -20.0827;
h1_openloop(1) = h01_openloop;
h2_openloop(1) = h02_openloop;
h1eq = 20.1685;
h2eq = 20.0827;

% System constant

S1 = 7.5*7.5;   % Cross-section area of Tank 1 (cm^2)
S2 = 10*10;     % Cross-section area of Tank 2 (cm^2)
g = 980;           % Gravitational constant (cm/s^2)
A1 = 0.0314;    % Cross-section outlet area of Tank 1 (m^2)
A2 = 0.07065;   % Cross-section outlet area of Tank 2 (m^2)
V = 39.84;

% System dimension
nx = 2;
nu = 1;
ny = 1;

% Loop constant
N = 600;
dt = 1;

% Vertis calculation
h1min = 1;
h1max = 23;
h2min = 1;
h2max = 21;

% Assume
A{1} = [ -A1/S1 * sqrt(2*g/h1min)                0;
          A1/S2 * sqrt(2*g/h1min)     -A2/S2 * sqrt(2*g/h2min)]
A{2} = [ -A1/S1 * sqrt(2*g/h1min)                0;
          A1/S2 * sqrt(2*g/h1min)     -A2/S2 * sqrt(2*g/h2max)]
A{3} = [ -A1/S1 * sqrt(2*g/h1max)                0;
          A1/S2 * sqrt(2*g/h1max)     -A2/S2 * sqrt(2*g/h2min)]
A{4} = [ -A1/S1 * sqrt(2*g/h1max)                0;
          A1/S2 * sqrt(2*g/h1max)     -A2/S2 * sqrt(2*g/h2max)]

nl = numel(A);

%%
% Discreatize of vertis

for i = 1:1:nl
    
    A{i} = A{i}*dt + eye(nx);
    
end

B = [1/S1 ; 0];
C = [1 1];
nl = numel(A);

% Constraints
%umax = 0.017368;
    umax = 6;
% Weighting matrices
Q = diag([1 1]);
R = 2e-5;

% Decision variable
O = sdpvar(nx,nx,'symmetric');
Y = sdpvar(nu,nx);
X = sdpvar(nu,nu);
T = sdpvar(ny,ny);
G = sdpvar(nx,nx);
gamma = sdpvar(1);

% Preallocate variables
h0 = [h01_openloop ; h02_openloop]
x = zeros(nx,N+1);
x(:,1) = h0
h_tank1(1) = h1_openloop(1);
h_tank2(1) = h2_openloop(1);

J_online = 0;

%% Optimization problem

tStart = cputime;

for k = 1:dt:N
    
    tStart2 = cputime;
    
    LMIs = [];
    LMIs = [LMIs, [1 x(:,k)'; x(:,k) O] >= 0];

    for i = 1:nl
        
        LMIs = [ LMIs, [...
         G+G'-O              G'*A{i}' + Y'*B'  G'*Q^(1/2)'     Y'*R^(1/2)';
         A{i}*G + B*Y   O                zeros(nx,nx)    zeros(nx,nu);
         Q^(1/2)*G      zeros(nx,nx)     gamma*eye(nx)   zeros(nx,nu);
         R^(1/2)*Y      zeros(nu,nx)     zeros(nu,nx)    gamma*eye(nu)] >= 0];   
        
    end
     
   LMIs = [ LMIs,[umax^2 Y; Y' G+G'-O] >= 0];

    optimize(LMIs,gamma)
    K = value(Y)/value(G);
    O_value{k} = value(G);
    K_value{k} = value(K);
    
        p = rand(1);
        r = rand(1);
        Ap = (r)*((p)*A{1}+(1-p)*A{3})+(1-r)*((p)*A{2}+(1-p)*A{4});

    %Plant
    norm_K(:,k)=norm(K,2);
    u(:,k) = K*x(:,k);
    x(:,k+1) = Ap*x(:,k) + B*u(:,k);
    y(:,k) = C*x(:,k);
    
    h_tank1(k) = x(1,k) + h1eq;
    h_tank2(k) = x(2,k) + h2eq;
    
    J_online = J_online + ((x(:,k+1)'*Q*x(:,k+1))+(u(:,k)'*R*u(:,k)));

    tEnd2 = cputime - tStart2
    t_online2(k) =  tEnd2;
    tStart2 = 0;
    tEnd2 = 0;
end

u_online = u;

h_tank1_online = h_tank1;
h_tank2_online = h_tank2;

 tEnd = cputime - tStart
 t_online =  tEnd;

%% Open loop

h01_openloop = 0;
h02_openloop = 0;
h1_openloop(1) = h01_openloop;
h2_openloop(1) = h02_openloop;
A1 = (22/7)*(0.505^2)/4;     
A2 = (22/7)*(0.505^2)/4;   

S1 = 7.5*8;  
S2 = 10*10;        
dt = 1;

% Loop constant

for i = 1:1:(N-1)
        
    H = TankCal(V,S1,S2,A1,A2,h1_openloop(i),h2_openloop(i));
    
    h1_openloop(i+1) = h1_openloop(i) + H(1);
    h2_openloop(i+1) = h2_openloop(i) + H(2);
    
end

%% plot result

t = 0:dt:N;
figure(1);  hold on; grid on;
plot(t,x(1,:),'Color',[0 0.4470 0.7410],'Linewidth',1.5);
plot(t,x(2,:),'Color',[0.8500 0.3250 0.0980],'Linewidth',1.5);
hold off;
xlabel('time (s)');
ylabel('h - h_s (cm)');
legend('Tank NO.1','Tank NO.2');

t = 0:dt:(N-1);
figure(2); 
plot(t,u,'Linewidth',1.5)
xlabel('time (s)');
ylabel('u-u_s');
grid on;

t = 0:dt:(N-1);
figure(3); 
subplot(2,1,1);
plot(t,h_tank1,'Color',[0 0.4470 0.7410],'Linewidth',1.5)
hold on
t = 0:dt:(N-1);
plot(t,h1_openloop,'--','Color',[0.9290 0.6940 0.1250],'Linewidth',1.5)
xlabel('time (s)');
ylabel('Level of tank NO.1 (cm)');
legend('Online LMI-based RMPC','Open-loop');
grid on
hold on

t = 0:dt:(N-1);
subplot(2,1,2);
plot(t,h_tank2,'Color',[0.8500 0.3250 0.0980],'Linewidth',1.5)
hold on
t = 0:dt:(N-1);
plot(t,h2_openloop,'--','Color',[0.9290 0.6940 0.1250],'Linewidth',1.5)
xlabel('time (s)');
ylabel('Level of tank NO.2 (cm)');
legend('Online LMI-based RMPC','Open-loop');
grid on
hold off


J_online

%% Save data

%save('Tank_Data_New','O_value','K_value','A','Ap','B','C','u_online','h1_openloop','h2_openloop','h_tank1_online','h_tank2_online','J_online','t_online');