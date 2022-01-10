%% Magnetorquer Control
% Clayton Jaksha
% This program aims to create transfer functions for each major component of
% the general cubesat setup, control the cubesat attitude with PID-driven
% magnetorquers, and optimize the PID setup each time a new attitude command
% is issued. Because of power limitations, the system will appear greatly
% underdamped and that is simply because it is almost wholly underdamped and
% our magnetorquers consume a great deal of the satellite's power budget.
% Therefore, teh magnetorquers cannot be left to draw large currents.
%% Direction Vector
qc=[1 0 0]'; %[x y z]
qc=qc/norm(qc);
%% Magnetorquer/Actuator Params
l=0.08; %m
r=0.002; %m
N=1000; %turns
L=(N^2*r^2)/(9*r+10*l);
R=100; %Ohms
Bmag=60E-6; %T
A=tf(N*pi*(r^2)*(9*r+10*l)*40,[(r^2)*(N^2) R*(9*r+10*l)]);
%% Initialize System Inertia Params
Vcc=12;
h=0.3; %m
w=0.1; %m
d=0.1; %m
m=4; %kg
Imx=[(m/12)*(d^2+h^2) 0 0; 0 (m/12)*(w^2+d^2) 0; 0 0 (m/12)*(w^2+d^2)];
Irot=norm(Imx*qc);
%% Test 90-deg Attitude Change - 4kg, 3u 
n_cycles=1000000;
t_step=0.1;
t=0;
% set initial conditions
th=pi/6;
th_d=0;
th_dd=0;
i_act=0;
%
ii=1;
t_data=zeros(1,n_cycles-1);
th_data=t_data;
th_c_data=t_data;
i_data=t_data;
pwr_data=t_data;
while ii<n_cycles
    th_d_k1=th_d;
    th_dd_k1=th_dd;
    i_act_k1=i_act;
    Vo=fuzz(th,th_d,th_dd,Irot);
    i_act=(t_step*Vo+L*i_act_k1)/(t_step*R+L);
    pwr=abs(Vo)*abs(i_act);
    torque=N*i_act*Bmag*pi*(r^2)*abs(sin(th));
    th_dd=(torque/Irot);
    th_d=th_d_k1+(th_dd+th_dd_k1)*(t_step/2);
    th=(th+(th_d+th_d_k1)*(t_step/2));
    t_data(ii)=t;
    th_c_data(ii)=0;
    th_data(ii)=th;
    i_data(ii)=i_act;
    pwr_data(ii)=pwr;
    t=t+t_step;
    ii=ii+1;
end
figure
subplot(3,1,1)
plot(t_data,th_data,t_data,th_c_data)
title('Spacecraft Attitude')
xlabel('Time [s]')
ylabel('Error Angle [rads]')
subplot(3,1,2)
plot(t_data,pwr_data,'r')
title('Control Unit Power Consumption')
xlabel('Time [s]')
ylabel('Power [W]')
subplot(3,1,3)
plot(t_data,i_data,'c')
title('Magnetorquer Current')
xlabel('Time [s]')
ylabel('Current [A]')
%% Re-initialize System Inertia Params
Vcc=12;
h=0.2; %m
w=0.1; %m
d=0.1; %m
m=2; %kg
Imx=[(m/12)*(d^2+h^2) 0 0; 0 (m/12)*(w^2+d^2) 0; 0 0 (m/12)*(w^2+d^2)];
Irot=norm(Imx*qc);
%% Test 90-deg Attitude Change - 2kg, 2u 
n_cycles=350000;
t_step=0.1;
t=0;
% set initial conditions
th=pi/6;
th_d=0;
th_dd=0;
i_act=0;
%
ii=1;
t_data=zeros(1,n_cycles-1);
th_data=t_data;
th_c_data=t_data;
i_data=t_data;
pwr_data=t_data;
while ii<n_cycles
    th_d_k1=th_d;
    th_dd_k1=th_dd;
    i_act_k1=i_act;
    Vo=fuzz(th,th_d,th_dd,Irot);
    i_act=(t_step*Vo+L*i_act_k1)/(t_step*R+L);
    pwr=abs(Vo)*abs(i_act);
    torque=N*i_act*Bmag*pi*(r^2)*abs(sin(th));
    th_dd=(torque/Irot);
    th_d=th_d_k1+(th_dd+th_dd_k1)*(t_step/2);
    th=(th+(th_d+th_d_k1)*(t_step/2));
    t_data(ii)=t;
    th_c_data(ii)=0;
    th_data(ii)=th;
    i_data(ii)=i_act;
    pwr_data(ii)=pwr;
    t=t+t_step;
    ii=ii+1;
end
figure
subplot(3,1,1)
plot(t_data,th_data,t_data,th_c_data)
title('Spacecraft Attitude')
xlabel('Time [s]')
ylabel('Error Angle [rads]')
subplot(3,1,2)
plot(t_data,pwr_data,'r')
title('Control Unit Power Consumption')
xlabel('Time [s]')
ylabel('Power [W]')
subplot(3,1,3)
plot(t_data,i_data,'c')
title('Magnetorquer Current')
xlabel('Time [s]')
ylabel('Current [A]')
%% Test 0.1-s Impulse Perturbation
n_cycles=900000;
t_step=0.01;
t=0;
% set initial conditions
th=0;
th_d=0;
th_dd=0;
i_act=0;
%
ii=1;
t_data=zeros(1,n_cycles-1);
th_data=t_data;
th_c_data=t_data;
i_data=t_data;
pwr_data=t_data;
while ii<n_cycles
    th_d_k1=th_d;
    th_dd_k1=th_dd;
    i_act_k1=i_act;
    Vo=fuzz(th,th_d,th_dd,Irot);
    i_act=(t_step*Vo+L*i_act_k1)/(t_step*R+L);
    pwr=abs(Vo)*abs(i_act);
    torque=N*i_act*Bmag*pi*(r^2)*abs(sin(th));
    th_dd=(torque/Irot)+heaviside(t-1)*heaviside(1.01-t)*(1E-4)/Irot; % heavisides create 0.05 N-m perturbation from t=[1,1.1]
    th_d=th_d_k1+(th_dd+th_dd_k1)*(t_step/2);
    th=th+(th_d+th_d_k1)*(t_step/2);
    t_data(ii)=t;
    th_c_data(ii)=0;
    th_data(ii)=th;
    i_data(ii)=i_act;
    pwr_data(ii)=pwr;
    t=t+t_step;
    ii=ii+1;
end
figure
subplot(3,1,1)
plot(t_data,th_data,t_data,th_c_data)
title('Spacecraft Attitude')
xlabel('Time [s]')
ylabel('Error Angle [rads]')
subplot(3,1,2)
plot(t_data,pwr_data,'r')
title('Control Unit Power Consumption')
xlabel('Time [s]')
ylabel('Power [W]')
subplot(3,1,3)
plot(t_data,i_data,'c')
title('Magnetorquer Current')
xlabel('Time [s]')
ylabel('Current [A]')
%% Test Low-Torque Constant Perturbation
n_cycles=200000;
t_step=0.1;
t=0;
% set initial conditions
th=0;
th_d=0;
th_dd=0;
i_act=0;
%
ii=1;
t_data=zeros(1,n_cycles-1);
th_data=t_data;
th_c_data=t_data;
i_data=t_data;
pwr_data=t_data;
while ii<n_cycles
    th_d_k1=th_d;
    th_dd_k1=th_dd;
    i_act_k1=i_act;
    Vo=fuzz(th,th_d,th_dd,Irot);
    i_act=(t_step*Vo+L*i_act_k1)/(t_step*R+L);
    pwr=abs(Vo)*abs(i_act);
    torque=N*i_act*Bmag*pi*(r^2)*abs(sin(th));
    th_dd=(torque/Irot)+(1E-8)/Irot;
    th_d=th_d_k1+(th_dd+th_dd_k1)*(t_step/2);
    th=th+(th_d+th_d_k1)*(t_step/2);
    t_data(ii)=t;
    th_c_data(ii)=0;
    th_data(ii)=th;
    i_data(ii)=i_act;
    pwr_data(ii)=pwr;
    t=t+t_step;
    ii=ii+1;
end
figure
subplot(3,1,1)
plot(t_data,th_data,t_data,th_c_data)
title('Spacecraft Attitude')
xlabel('Time [s]')
ylabel('Error Angle [rads]')
subplot(3,1,2)
plot(t_data,pwr_data,'r')
title('Control Unit Power Consumption')
xlabel('Time [s]')
ylabel('Power [W]')
subplot(3,1,3)
plot(t_data,i_data,'c')
title('Magnetorquer Current')
xlabel('Time [s]')
ylabel('Current [A]')
%% Test Tumble Recovery - 2kg, 2u 
n_cycles=500000;
t_step=0.1;
t=0;
% set initial conditions
th=0;
th_d=0.3;
th_dd=0;
i_act=0;
%
ii=1;
t_data=zeros(1,n_cycles-1);
th_d_data=t_data;
th_c_data=t_data;
i_data=t_data;
pwr_data=t_data;
while ii<n_cycles
    th_d_k1=th_d;
    th_dd_k1=th_dd;
    i_act_k1=i_act;
    Vo=fuzz(th,th_d,th_dd,Irot);
    i_act=(t_step*Vo+L*i_act_k1)/(t_step*R+L);
    pwr=abs(Vo)*abs(i_act);
    torque=N*i_act*Bmag*pi*(r^2)*abs(sin(th));
    th_dd=(torque/Irot);
    th_d=th_d_k1+(th_dd+th_dd_k1)*(t_step/2);
    th=(th+(th_d+th_d_k1)*(t_step/2));
    t_data(ii)=t;
    th_c_data(ii)=0;
    th_d_data(ii)=th_d;
    i_data(ii)=i_act;
    pwr_data(ii)=pwr;
    t=t+t_step;
    ii=ii+1;
end
figure
subplot(3,1,1)
plot(t_data,th_d_data,t_data,th_c_data)
title('Spacecraft Rotation')
xlabel('Time [s]')
ylabel('Spin Rate [rad/s]')
subplot(3,1,2)
plot(t_data,pwr_data,'r')
title('Control Unit Power Consumption')
xlabel('Time [s]')
ylabel('Power [W]')
subplot(3,1,3)
plot(t_data,i_data,'c')
title('Magnetorquer Current')
xlabel('Time [s]')
ylabel('Current [A]')