function PID
%% Function PID control: Simulates PID control of the FREE
% Created: June 2019 by Soheil Habibian
% Latest Revision: October 4, 2019
% Remove all variables from the workspace
clc;
% clear open figure windows
clf;
%% Establish global variables
global Gamma L R m_l I_l M_l F_l ke kt ce ct kp kd ki Pmax t dphi ddphi dphi_int
%% Constants (based on values in Clayton's report)
Gamma = (-40)*pi/180;   % fiber angle of relaxed FREE [rad]
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
kp = 10342;%(0.6)*17236.9;      % porpotional gain [Pa/rad]
ki = 94803;% 5.5*17236.9;       % porpotional gain [Pa/rad-s]
kd = 0;%0.15*17236.9;                % derivative gain [Pa-s/rad]
%% Trajectory planning parameters
phi_f1 = 50*pi/180;    % desired rotation [rad]
phi_f2 = 20*pi/180;    % desired rotation [rad]
phi_f3 = 80*pi/180;    % desired rotation [rad]
%% Perform Simlation
inc = 9;                % length of the whole simulation
tspan = [0 inc];
opts = odeset('RelTol',1e-6,'AbsTol',1e-6);
[t,y] = ode45(@freefunction,tspan,[0 0 0 0 0],opts);
%% Create first figure of results
% figure('NumberTitle','off',...
%     'Position',[100 150 900 400])

% Calculate pressure for plotting based on controller (P must be > 0)
N = length(t);
Pplot = zeros(1,N);
dphi = zeros(1,N);
ddphi = zeros(1,N);
dphi_int = zeros(1,N);

for ii=1:N
    if (0 < t(ii)) && (t(ii) <= inc/3)                    % 1st step starts here
        dphi(ii) = phi_f1;
        ddphi(ii) = 0;
        dphi_int(ii) = phi_f1*(t(ii));

    elseif (inc/3 < t(ii)) && (t(ii) <= 2*inc/3)          % 2nd step starts here

         dphi(ii) = phi_f2;
         ddphi(ii) = 0;
         dphi_int(ii) = phi_f2*(t(ii)-inc/3)...
             + phi_f1*(inc/3);

    else                                       % 3rd step starts here

        dphi(ii) = phi_f3;
        ddphi(ii) = 0;
        dphi_int(ii) = phi_f3*(t(ii)-2*inc/3)...
            + phi_f2*(inc/3)...
            + phi_f1*(inc/3);

    end
    
   Pplot(ii) = kp*(dphi(ii)-y(ii,2)) + kd*(ddphi(ii)-y(ii,4))+ ki*(dphi_int(ii)-y(ii,5));  % PID control of Pressure
if Pplot(ii) < 0
    Pplot(ii) = 0;
elseif Pplot(ii) > Pmax*6894.76
    Pplot(ii) = Pmax*6894.76;
end
    
end

% Plot rotation response and trajectory
subplot(1,3,1)
plot(t,y(:,2)*180/pi,'LineWidth',2);
hold on

line([0,inc/3],[phi_f1*180/pi,phi_f1*180/pi],'Color','red');
line([inc/3,inc/3],[phi_f1*180/pi,phi_f2*180/pi],'Color','red');
line([inc/3,2*inc/3],[phi_f2*180/pi,phi_f2*180/pi],'Color','red');
line([2*inc/3,2*inc/3],[phi_f2*180/pi,phi_f3*180/pi],'Color','red');
line([2*inc/3,inc],[phi_f3*180/pi,phi_f3*180/pi],'Color','red');

xlabel('Time (s)');	
ylabel('Rotation (\circ)');
legend('System response','Setpoint','Location', 'Best')
grid on; grid minor
axis([0 9 0 90]);

% Plot elongation response and trajectory
subplot(1,3,2)
plot(t,y(:,1)*1000,'LineWidth',2);
xlabel('Time (s)');	
ylabel('Elongation (mm)');
grid on; grid minor
axis([0 9 -0.7 0.1]);

% Plot pressure
subplot(1,3,3)
plot(t,Pplot/6894.76,'LineWidth',2);
xlabel('Time (s)');
ylabel('Pressure (psi)')
grid on; grid minor;
hold on
axis([0 9 0 10])

function ydot = freefunction(t,y)
%%  freefunction(t,y): Simulating the dynamic response of a FREE
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
%% Calculate pressure based on controller (P must be non-negative)
if (0 < t) && (t <= inc/3)                    % 1st step starts here
    dphi = phi_f1;
    ddphi = 0;
    dphi_int = phi_f1*(t);
    
elseif (inc/3 < t) && (t <= 2*inc/3)          % 2nd step starts here
    
     dphi = phi_f2;
     ddphi = 0;
     dphi_int = phi_f2*(t-inc/3)...
         + phi_f1*(inc/3);

else          % 3rd step starts here

    dphi = phi_f3;
    ddphi= 0;
    dphi_int = phi_f3*(t-2*inc/3)...
         + phi_f2*(inc/3)...
         + phi_f1*(inc/3);
        
end


P = kp*(dphi-y(2)) + kd*(ddphi-y(4))+ ki*(dphi_int-y(5));  % PID control of Pressure
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
