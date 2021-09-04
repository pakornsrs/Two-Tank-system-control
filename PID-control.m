% PID control

clear all
clc

    
ysp = 20.0827;
yt(1) = 0;
y0 = yt(1);


S = 7.5*7.5;   % Cross-section area of Tank 1 (cm^2)
g = 980;           % Gravitational constant (cm/s^2)
A = (22/7)*(0.505^2)/4;        % Cross-section outlet area of Tank 1 (m^2)
u_s = 39.84;
u = 0;

er(1) = ysp - yt(1);

Kc = 3;
Ti = 35;
Td = 1;


dt = 1;


N = 600;

for i = 1:1:N
    
    % Function
    dh(i) = (-A*sqrt(2*g*yt(i))/S)+u/(S);
    yt(i+1) = yt(i)+dh(i);
    yref(i+1) = ysp-yt(i+1);
    
    % Integrate
    er(i+1) = ysp - yt(i+1);
    b = i; a = i-1;
    I(i) = (b-a)*((er(i+1)+er(i))/2);
    SumI = sum(I);
    
    % Differentation 
    der(i) = (er(i+1) - er(i))/i
    
    
    u_pid(i) = Kc*(er(i+1) + (1/Ti)*SumI + Td*der(i));
    %if u_pid(i) > 45
    %    u_pid(i) = 45;
    %end
    u = u_pid(i)
end
%%

t=0:1:N;
tu=0:1:N-1;

figure(1)
subplot(2,1,1)
con_max = (23)*ones(1,N);
plot(tu,con_max,'r','Linewidth',1)
hold on
con_s = (20.0827)*ones(1,N);
plot(tu,con_s,'--r','Linewidth',1)
hold on
plot(t,yt,'Linewidth',1.5)
legend('Hard constraint','Steady-state');
hold on
ylabel('Level(cm3/s)')
xlabel('time(s)')
grid on

figure(2)
con_max = (45)*ones(1,N);
plot(tu,con_max,'r','Linewidth',1)
hold on
con_s = (u_s)*ones(1,N);
plot(tu,con_s,'--r','Linewidth',1)
hold on
plot(tu,u_pid,'Linewidth',1.5)
hold on
ylabel('Flow rate(cm3/s)')
xlabel('time(s)')
legend('Hard constraint','Steady-state');
grid on

%%
load('All_data.mat')

figure(3); 
ty=0:1:N;
plot(tu,h_tank1_offline(1:1:600),'Linewidth',1.5)
hold on
plot(tu,h_tank1_online(1:1:600),'Linewidth',1.5)
t = 0:dt:(N-1);
plot(tu,h1_openloop(1:1:600),'Linewidth',1.5)
hold on
plot(ty,yt,'Linewidth',1.5)
con_max = (23)*ones(1,N);
plot(tu,con_max,'r','Linewidth',1)
hold on
xlabel('time (s)');
ylabel('Level of tank NO.1 (cm)');
legend('Offline MPC','Online MPC','Open-loop','PID');
grid on



