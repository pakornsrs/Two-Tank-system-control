clear all
clc

%% Open loop

% Initial hight of the tank
h01 = 0;
h02 = 0;
h1(1) = h01;
h2(1) = h02;

% System constant
PWM = 80;
%V = (0.1512*PWM) - 4.2712;  
%V = (-1e-6*PWM^4) + (0.0004*PWM^3) - (0.04*PWM^2) +(1.7938*PWM) +7.1778;
V = 39.84;

S1 = 7.5*8;  
S2 = 10*10;        
A1 = (22/7)*(0.505^2)/4;     
A2 = (22/7)*(0.505^2)/4;   

% Loop constant
N = 600;

for i = 1:1:(N-1)
        
    H = tank(V,S1,S2,A1,A2,h1(i),h2(i));
    
    
    h1(i+1) = h1(i) + H(1);
    h2(i+1) = h2(i) + H(2);
    
end

%% Plot

t = 0:1:(N-1);
subplot(2,1,1);
plot(t,h1,'r','Linewidth',1.5);
xlabel('time');
ylabel('level (cm)');
legend('tank 1');
grid on
hold on
subplot(2,1,2);
plot(t,h2,'Linewidth',1.5);
xlabel('time');
ylabel('level (cm)');
legend('tank 2');
grid on
hold on

[h2']

