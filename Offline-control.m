clear all
clc

load('Tank_Data_New')
load('experiment')
%%
h1_initial = -20.1685;
h2_initial =  -20.0827;
h1eq = 20.1685;
h2eq = 20.0827;
J_offline = 0;

% Weighting matrices
Q = diag([1 1]);
R = 2e-5;

% System dimension
nx = 2;
nu = 1;
ny = 1;

% Loop constant
N = 800;
dt = 1;

% selected ellipsoid
el = [2,4,6,8,10,15,30,45,60,90,120,160,200];
N_offline = numel(el);

% Preallocate variables
x = zeros(nx,N+1);
K = cell(1,N);

x0 = [h1_initial ; h2_initial];
x(:,1) = x0;

% Loop

tStart = cputime;

for k = 1:N;
    
    tStart2 = cputime;
    
    for i=1:N_offline       
        
        e = el(i);
        val = x(:,k)'*inv(O_value{e})*x(:,k);
        
        if val >1
            break 
            
        end        
    end
 
    if i == N_offline  
      
        i_k = i;   
    
    else
        
        i_k = i-1;
       
        if i_k == 0;
            i_k = 1;
        
        end
    end
    
    % Plant
    
        p = rand(1);
        r = rand(1);
        Ap = (r)*((p)*A{1}+(1-p)*A{3})+(1-r)*((p)*A{2}+(1-p)*A{4});
        
    K{k} = K_value{i_k};      
    u(:,k) = K{k}*x(:,k);
    x(:,k+1) = Ap*x(:,k) + B*u(:,k);
    y(:,k) = C*x(:,k);
    
    h_tank1(k+1) = x(1,k) + h1eq;
    h_tank2(k+1) = x(2,k) + h2eq;
    
     J_offline = J_offline + ((x(:,k+1)'*Q*x(:,k+1))+(u(:,k)'*R*u(:,k)));

    tEnd2 = cputime - tStart2
    t_offline2(k) =  tEnd2;
    tStart2 = 0;
    tEnd2 = 0;

end
mean(t_offline2)
% plot results

t = 0:dt:N;
figure(1);  hold on; grid on;
plot(t,x(1,:),'Color',[0.8500 0.3250 0.0980],'Linewidth',1.5);
plot(t,x(2,:),'Color',[0 0.4470 0.7410],'Linewidth',1.5);
hold off;
xlabel('time (s)');
ylabel('h - h_s (cm)');
legend('Tank NO.1','Tank NO.2');

t = 0:dt:(N-1);
figure(2); 
plot(t,u,'Linewidth',1.5);
xlabel('time (s)');
ylabel('u-u_s');
grid on;

%%
t = 0:dt:(N);
t_online = 0:1:799;
figure(3); 
subplot(2,1,1);
plot(t,h_tank1,'Linewidth',1.5)
hold on
plot(t_online,h_tank1_online,'Linewidth',1.5)
t = 0:dt:(N-1);
plot(t,h1_openloop,'Linewidth',1.5)
xlabel('time (s)');
ylabel('Level of tank NO.1 (cm)');
legend('Offline MPC','Online MPC','Open-loop');
grid on
hold on

t = 0:dt:(N);
subplot(2,1,2);
plot(t,h_tank2,'Linewidth',1.5)
hold on
plot(t_online,h_tank2_online,'Linewidth',1.5)
hold on
t = 0:dt:(N-1);
plot(t,h2_openloop,'Linewidth',1.5)
xlabel('time (s)');
ylabel('Level of tank NO.2 (cm)');
legend('Offline MPC','Online MPC','Open-loop');
grid on
hold off
%%
t = 0:dt:(N-1);
figure(5);
subplot(1,2,1)
plot(t,u_online,'Color',[0.8500 0.3250 0.0980],'Linewidth',1.5);
legend('Online algorithm');
xlabel('time (s)');
ylabel('u-u_s');
grid on;
subplot(1,2,2)
plot(t,u,'Linewidth',1.5);
hold on
plot(t,u_online,'--','Color',[0.8500 0.3250 0.0980],'Linewidth',1.5)
legend('Offline algorithm','Online algorithm');
xlabel('time (s)');
ylabel('u-u_s');
grid on;

 tEnd = cputime - tStart;
 t_offline = tEnd

%% Ellipsoid
xvar = sdpvar(nx,1);

% Different 
%% Ellipsoid
xvar = sdpvar(nx,1);

%Ellipsoid 1
    cont = [[xvar'*inv(O_value{1})*xvar]<=1]
    ops = sdpsettings('plot.shade',0.1); %1 mean %visturl
    plot_col = [1 0.85 0.4]; 
    figure(4);
    plot(cont,xvar,plot_col,200,ops)
    hold on
%Ellipsoid 2
    cont = [[xvar'*inv(O_value{15})*xvar]<=1]
    ops = sdpsettings('plot.shade',0.1); %1 mean %visturl
    plot_col = [1 0.85 0.4]; 
    plot(cont,xvar,plot_col,200,ops)
    hold on
    
%{
    %Ellipsoid Extra
    cont = [[xvar'*inv(O_value{4})*xvar]<=1]
    ops = sdpsettings('plot.shade',0.1); %1 mean %visturl
    plot_col = [1 0.85 0.4]; 
    plot(cont,xvar,plot_col,200,ops)
    hold on

    cont = [[xvar'*inv(O_value{6})*xvar]<=1]
    ops = sdpsettings('plot.shade',0.1); %1 mean %visturl
    plot_col = [1 0.85 0.4]; 
    plot(cont,xvar,plot_col,200,ops)
    hold on
    
    cont = [[xvar'*inv(O_value{8})*xvar]<=1]
    ops = sdpsettings('plot.shade',0.1); %1 mean %visturl
    plot_col = [1 0.85 0.4]; 
    plot(cont,xvar,plot_col,200,ops)
    hold on
    
    cont = [[xvar'*inv(O_value{10})*xvar]<=1]
    ops = sdpsettings('plot.shade',0.1); %1 mean %visturl
    plot_col = [1 0.85 0.4]; 
    plot(cont,xvar,plot_col,200,ops)
    hold on
    %}
    
%Ellipsoid 3
    cont = [[xvar'*inv(O_value{30})*xvar]<=1]
    ops = sdpsettings('plot.shade',0.1); %1 mean %visturl
    plot_col = [1 0.85 0.4]; 
    plot(cont,xvar,plot_col,200,ops)
    hold on
%Ellipsoid 4
    cont = [[xvar'*inv(O_value{45})*xvar]<=1]
    ops = sdpsettings('plot.shade',0.1); %1 mean %visturl
    plot_col = [1 0.85 0.4]; 
    plot(cont,xvar,plot_col,200,ops)
    hold on
%Ellipsoid 5
    cont = [[xvar'*inv(O_value{60})*xvar]<=1]
    ops = sdpsettings('plot.shade',0.1); %1 mean %visturl
    plot_col = [1 0.85 0.4]; 
    plot(cont,xvar,plot_col,200,ops)
    hold on
%Ellipsoid 6
    cont = [[xvar'*inv(O_value{90})*xvar]<=1]
    ops = sdpsettings('plot.shade',0.1); %1 mean %visturl
    plot_col = [1 0.85 0.4]; 
    plot(cont,xvar,plot_col,200,ops)
    hold on
%Ellipsoid 7
    cont = [[xvar'*inv(O_value{120})*xvar]<=1]
    ops = sdpsettings('plot.shade',0.1); %1 mean %visturl
    plot_col = [1 0.85 0.4]; 
    plot(cont,xvar,plot_col,200,ops)
    hold on
%Ellipsoid 8
    cont = [[xvar'*inv(O_value{160})*xvar]<=1]
    ops = sdpsettings('plot.shade',0.05); %1 mean %visturl
    plot_col = [1 0.85 0.4]; 
    plot(cont,xvar,plot_col,200,ops)
    hold on
%Ellipsoid 9
    cont = [[xvar'*inv(O_value{200})*xvar]<=1]
    ops = sdpsettings('plot.shade',0.05); %1 mean %visturl
    plot_col = [1 0.85 0.4]; 
    plot(cont,xvar,plot_col,200,ops)
    hold on
    grid on
         xlabel('h_1-h_1_s (cm)');
         ylabel('h_2-h_2_s (cm)');
%

x1_plot = x1_exp - h1eq;
x2_plot = x2_exp - h2eq;
    
         plot(x1_plot,x2_plot,'*','Linewidth',0.5);
         xlabel('h_1-h_1_s (cm)');
         ylabel('h_2-h_2_s (cm)');
         hold on
         grid on
%%

    J_online
    J_offline
    
     t_online
     t_offline
%% error check

for i = 1:1:N

    h1_error(i) = abs(h_tank1_online(i) - h_tank1(i))/h_tank1_online(i) *100;
    h2_error(i) = abs(h_tank2_online(i) - h_tank2(i))/h_tank2_online(i) *100;
    
end

t = 1:dt:(N-2);
figure(5);
subplot(2,1,1);
plot(t,h1_error(3:1:N),'Color',[0.8500 0.3250 0.0980],'Linewidth',1.5)
xlabel('time (s)');
ylabel('%different');
legend('%Different of offline compare with online algorithm in tank 1');
grid on
hold on

subplot(2,1,2);
plot(t,h2_error(3:1:N),'Color',[0 0.4470 0.7410],'Linewidth',1.5)
xlabel('time (s)');
ylabel('%different');
legend('%Different of offline compare with online algorithm in tank 2');
grid on
hold on

%% Save zone

h_tank1_offline = h_tank1;
h_tank2_offline = h_tank2;
dh_offline = x;

%save('All_Data','O_value','K_value','h1_openloop','h2_openloop','h_tank1_online','h_tank2_online','h_tank1_offline','h_tank2_offline','J_online','t_online','J_offline','t_offline','dh_offline');
%save('experiment','x1_exp','x2_exp')