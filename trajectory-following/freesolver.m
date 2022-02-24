function [t,y,Pplot,phiplot] = freesolver(elon1,thet1,thet2,t_f,inc,g1,g2)
%% Program freesolver.m: M-file to simulate the dynamic response of a FREE
% Created: June 2019 by Soheil Habibian
% Remove all variables from the workspace
clc;
% clear open figure windows
clf;
%% Establish global variables
global Gamma L R m_l I_l dphi M_l F_l ke kt ce ct kp kd ki Pmax t
%% Constants (based on values in Clayton's report)
Gamma = (40)*pi/180;   % fiber angle of relaxed FREE [rad]
L = 0.12;               % length of relaxed FREE [m]
R = 0.007;              % radius of relaxed FREE [m]
m_l = 0.028;            % mass of end cap [kg]
I_l = m_l*R^2;          % mass moment of inertia of end cap [kg.m^2]
Pmax = 10;      % Max pressure [psi]
%% External load and torque
M_l = 0;                % external torque
F_l = m_l*9.81;         % external force
%% Stiffness and damping factors of FREE
ke = 16478;           % FREE's stiffness [N/m]
kt = 0.0862;           % FREE's torsional stiffness [Nm/rad]
ce = 0.34;                 % FREE's axial stiffness [N-s/m]
ct = 0.0000397;             % FREE's torsional damping constant [Nm-s/rad]
%% PID control gains
kp = g1; kd = 0; ki = g2;  % Control gains

%% Trajectory planning parameters
phi_i = -thet1*pi/180;       % initial angle of rotation, first step [rad]
phi_f = -(thet2)*pi/180;    % final rotation [rad]

t_d = t_f; % desired time to reach the set point angle of rotation [s]

%% Perform Simlation
tend = inc;
tspan = [0 tend];
opts = odeset('RelTol',1e-6,'AbsTol',1e-6);
[t,y] = ode45(@freefunction,tspan,[elon1 phi_i 0 0 0],opts);

%% Create first figure of results
% figure('NumberTitle','off',...
%     'Position',[50 150 1000 500])

% % Plot elongation
% subplot(2,3,2)
% plot(t,y(:,1)*1000,'LineWidth',0.8);
% xlabel('time[s]');	
% ylabel('elongation [mm]');
% grid on; grid minor
% hold on
% % axis([0 0.35 -20 10])

% Calculate pressure for plotting based on controller (P must be > 0)
N = length(t);
Pplot = zeros(1,N);
for i=1:N
    Pplot(i) = -(kp*(dphi-y(i,2)) - kd*y(i,4) + ki*(dphi*t(i) - y(i,5)));  % PID control of Pressure
if Pplot(i) < 0
    Pplot(i) = 0;
elseif Pplot(i) > Pmax*6894.76
    Pplot(i) = Pmax*6894.76;
end
end

phiplot = zeros(1,N);
for ii=1:N
    if t(ii) < t_d
    phiplot(ii) = phi_i +(t(ii)^2)*3*(phi_f - phi_i)/(t_d^2) - (t(ii)^3)*2*(phi_f - phi_i)/(t_d^3);  % PID control of Pressure
    else
        phiplot(ii) = dphi;
    end
end

% % Plot rotation
% subplot(1,2,2)
% % subplot(2,3,[4 5])
% plot(t,y(:,2)*180/pi,'LineWidth',1.2);
% hold on
% plot(t,phiplot*180/pi,'LineWidth',1.2);
% hold on
% line([0,t(end)],[phi_f*180/pi,phi_f*180/pi],'Color','red');
% xlabel('time[s]');	
% ylabel('\theta^{\circ}');
% axis([0 t(end) phi_f*180/pi-5 0])
% legend('system response','cubic polynomial')
% grid on; grid minor

% % Plot pressure
% subplot(1,2,1)
% plot(t,Pplot/6894.76,'LineWidth',0.8);
% xlabel('time[s]');
% ylabel('pressure [psi]')
% grid on; grid minor;
% hold on
% % axis([0 0.35 0 10])

% % Calculate fiber angle and tube radius for plotting
% gammaplot = zeros(1,N);
% rplot = zeros(1,N);
% for i=1:N
%     gammaplot(i) = acos((L+y(i,1))*cos(Gamma)/L);                        % deformed fiber angle
%     rplot(i) = L*tan(gammaplot(i))/((L*tan(Gamma)/R) + y(i,2));          % deformed radius
% end
% 
% % Plot FREE's fiber angle
% subplot(2,3,3)
% plot(t,gammaplot*180/pi,'LineWidth',0.8);
% xlabel('time[s]');	
% ylabel('\gamma^{\circ}');
% grid on; grid minor
% hold on
% % axis([0 0.35 0 45])
% 
% % Plot FREE's radius
% subplot(2,3,6)
% plot(t,rplot*1000,'LineWidth',0.8);
% xlabel('time[s]');	
% ylabel('radius [mm]');
% grid on; grid minor
% hold on
% % axis([0 0.35 -20 0])

function ydot = freefunction(t,y)
% Function freefunction(t,y): function used by freesolver.m in simulating
% the dynamic response of a FREE
% Created: October 2018 by Soheil Habibian
% Latest Revision: November 17, 2018 by Keith W. Buffinton
%% variables S(elongation), phi(rotation), derivatives, and integral
% y(1) = s
% y(2) = phi
% y(3) = sdot
% y(4) = phidot
% y(5) = phiint

ydot(1)= y(3);
ydot(2)= y(4);
ydot(5)= y(2);

%% Trajectory planning Cubic polynomial
if t < t_d
dphi = phi_i + ((t^2)*3*(phi_f - phi_i)/(t_d^2)) - ((t^3)*2*(phi_f - phi_i)/(t_d^3));
ddphi = t*6*(phi_f - phi_i)/(t_d^2) - (t^2)*6*(phi_f - phi_i)/(t_d^3);
else
    dphi = phi_f;
    ddphi = 0;
end
%% Calculate pressure based on controller (P must be non-negative)
P = -(kp*(dphi-y(2)) - kd*(ddphi-y(4)) + ki*(dphi*t-y(5)));  % PID control of Pressure
if P < 0
    P = 0;
elseif P > Pmax*6894.76
    P = Pmax*6894.76;
end

%% Geometrical relationships
gamma = acos((L+y(1))*cos(Gamma)/L);       % deformed fiber angle
r = L*tan(gamma)/(L*tan(Gamma)/R + y(2));  % deformed radius

%% Elastomer's force and torque
F_e = -ke*y(1) - ce*y(3);                  % elastomer force (stiffness)
M_e = -kt*y(2) - ct*y(4);                  % elastomer torque (torsional stiffness)

%% Differential equations
ydot(3) = (P*pi*r^2*(1-2*cot(gamma)^2) + F_l + F_e)/m_l;
ydot(4) = (-2*P*pi*r^3*cot(gamma) + M_l + M_e)/I_l;
ydot=ydot';
end
end
