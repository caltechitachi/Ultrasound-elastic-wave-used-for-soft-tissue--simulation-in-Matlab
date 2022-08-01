clc
clear all;
close all;

mu1 = 650*ones(1,43); 
eta = 0.1*ones(1,43);
for i= 21:43
  mu1(i) = 800;
  eta(i) = 0.2;
end

r=3e-4:3e-4:43*3e-4; %tia r co toa do tu 0.3 mm den 12.9 mm; chia lam 43 diem

frq=100;             %tan so rung cua kim 100 Hz
w=2*pi*frq;          %tan so goc cua kim
rho=1000;            %mat do khoi cua moi truong 1000 kg/m3
r0=2e-4;             %r0
%r0px=6e-3;            
dr=3e-4;             %delta r
dt=6/10000;          %delta t
phi_0=0; %-0.377;    %phi ban dau
A=0.008;            %bien do
n=400;
k=zeros(1,43);    %khoi tao ma tran "so song phuc"
alfa=zeros(1,43); %khoi tao ma tran "he so suy giam"
ks=zeros(1,43);   %khoi tao ma tran "so song"

for i=1:43    
        k(i)=sqrt(rho*w^2/(mu1(i)-1i*w*eta(i)));
        ks(i)=real(k(i));
        alfa(i)=-imag(k(i));
end
% -------xay dung ham van toc theo phuong trinh song-----------------
v=zeros(43,n);
for i=1:43
    for k=1:n
        v(i,k)= (1/sqrt(r(i)-r0))*A*cos(w*k*dt+ks(i)*(r(i)-r0)-phi_0);
    end
end
%==========================================================================

% -------xay dung ham van toc phan xa-----------------
vpx=zeros(43,n);
for i=1:20
    for k=1:n
        vpx(i,k)= (1/sqrt(r(21)-r(i)))*0.2*A*cos(w*k*dt-ks(i)*(r(21)-r(i))-phi_0);
    end
end
%==========================================================================
for i=1:43
   
    vth= v(i)+vpx(i);
    
    
end
figure(1);
hold on;
grid on;
y1=plot(ks,'--k');                 % do thi ks ly tuong
xlabel('Spatial location');
ylabel('WaveNumber [m^-^1]');
set(y1,'LineWidth',3);
legend('Ideal');
%-------------------------------------------
figure(2);
hold on;
grid on;
y1=plot(alfa,'--k');                 % He so suy giam ly tuong
xlabel('Spatial location');
ylabel('Attenuation [Np m^-^1]');
set(y1,'LineWidth',3);
legend('Ideal');
%-------------------------------------------
figure(3);
hold on;
grid on;
plot(v(15,:));      %van toc ly tuong theo thoi gian tai diem 15
title('Ideal velocity');
xlabel('Time steps');
ylabel('Velocity [m s^-^1]');
%-------------------------------------------
%---------- Do thi van toc theo khong gian khi dung ptr truyen song ------
figure(4);
for t = 25:2:n
    plot(v(:,t),'r');  
    axis([0 45 -1 1]);
    pause(.1);
end
grid on;
xlabel('Spatial location');
ylabel('Velocity [m s^-^1]');
%---------- Do thi van toc theo khong gian khi dung ptr truyen song song phan xa ------
figure(5);
for t = 25:2:n
    plot(vpx(:,t),'r');  
    axis([0 45 -1 1]);
    pause(.1);
end
grid on;
xlabel('Spatial location');
ylabel('Velocity [m s^-^1]');
%----------cong tin hieu ------
figure(6);
for t = 25:2:n
    plot(vth(:,t),'r');  
    axis([0 45 -1 1]);
    pause(.1);
end
grid on;
xlabel('Spatial location');
ylabel('Velocity [m s^-^1]');
