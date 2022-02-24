 function trajectory
%% Function trajectory: Simulates the trajectory following of the FREE
% Created: June 2019 by Soheil Habibian
% Latest Revision: October 3, 2019
% Remove all variables from the workspace
clc;
% clear open figure windows
clf;
%% Establish global variables
global Gamma L R m_l I_l M_l F_l ke kt ce ct kp kd ki Pmax t dphi ddphi phiplot dphi_int
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
kp = (1)*17236.9;      % porpotional gain [Pa/rad]
ki = 35*17236.9;       % porpotional gain [Pa/rad-s]
kd = 0;                % derivative gain [Pa-s/rad]
%% Trajectory planning parameters
phi_i = 0;       % initial angle of rotation, first step [rad]
phi_f1 = 40*pi/180;    % desired rotation [rad]
phi_f2 = 10*pi/180;    % desired rotation [rad]
phi_f3 = 70*pi/180;    % desired rotation [rad]
%% Perform Simlation
inc = 9;                % length of the whole simulation
t_d = 1.5;                % desired time to reach the set point angle of rotation [s]
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
    if (t(ii) <= t_d)                                   % 1st step starts here

        dphi(ii) = phi_i + 3*(phi_f1 - phi_i)*(t(ii)^2)/(t_d^2) - 2*(phi_f1 - phi_i)*(t(ii)^3)/(t_d^3);
        ddphi(ii) = 6*(phi_f1 - phi_i)*t(ii)/(t_d^2) - 6*(phi_f1 - phi_i)*(t(ii)^2)/(t_d^2);
        dphi_int(ii) = phi_i*t(ii) + (phi_f1 - phi_i)*(t(ii)^3)/(t_d^2) - (phi_f1 - phi_i)*(t(ii)^4)/((t_d^3)*2);

    elseif (t_d < t(ii)) && (t(ii) <= inc/3)      

        dphi(ii) = phi_f1;
        ddphi(ii) = 0;
        dphi_int(ii) = phi_f1*(t(ii)-t_d)...
            + phi_i*t_d + (phi_f1 - phi_i)*(t_d^3)/(t_d^2) - (phi_f1 - phi_i)*(t_d^4)/((t_d^3)*2);

    elseif (inc/3 < t(ii)) && (t(ii) <= inc/3+t_d)          % 2nd step starts here

        dphi(ii) = phi_f1 + ((t(ii)-inc/3)^2)*3*(phi_f2 - phi_f1)/(t_d^2) - ((t(ii)-inc/3)^3)*2*(phi_f2 - phi_f1)/(t_d^3);
        ddphi(ii) = (t(ii)-inc/3)*6*(phi_f2 - phi_f1)/(t_d^2) - ((t(ii)-inc/3)^2)*6*(phi_f2 - phi_f1)/(t_d^3);
        dphi_int(ii) = phi_f1*(t(ii)-inc/3) + (phi_f2 - phi_f1)*(t(ii)-inc/3)^3/(t_d^2) - (phi_f2 - phi_f1)*(t(ii)-inc/3)^4/((t_d^3)*2)...
            + phi_f1*(inc/3-t_d)...
            + phi_i*t_d + (phi_f1 - phi_i)*(t_d^3)/(t_d^2) - (phi_f1 - phi_i)*(t_d^4)/((t_d^3)*2);

    elseif (inc/3+t_d < t(ii)) && (t(ii) <= 2*inc/3)        

         dphi(ii) = phi_f2;
         ddphi(ii) = 0;
         dphi_int(ii) = phi_f2*(t(ii)-inc/3-t_d)...
            + phi_f1*(t_d) + (phi_f2 - phi_f1)*(t_d)^3/(t_d^2) - (phi_f2 - phi_f1)*(t_d)^4/((t_d^3)*2)...
            + phi_f1*(inc/3-t_d)...
            + phi_i*t_d + (phi_f1 - phi_i)*(t_d^3)/(t_d^2) - (phi_f1 - phi_i)*(t_d^4)/((t_d^3)*2);

    elseif (2*inc/3 < t(ii)) && (t(ii) <= 2*inc/3+t_d)          % 3rd step starts here

        dphi(ii) = phi_f2 + ((t(ii)-2*inc/3)^2)*3*(phi_f3 - phi_f2)/(t_d^2) - ((t(ii)-2*inc/3)^3)*2*(phi_f3 - phi_f2)/(t_d^3);
        ddphi(ii) = (t(ii)-2*inc/3)*6*(phi_f3 - phi_f2)/(t_d^2) - ((t(ii)-2*inc/3)^2)*6*(phi_f3 - phi_f2)/(t_d^3);
        dphi_int(ii) = phi_f2*(t(ii)-2*inc/3) + (phi_f3 - phi_f2)*(t(ii)-2*inc/3)^3/(t_d^2) - (phi_f3 - phi_f2)*(t(ii)-2*inc/3)^4/((t_d^3)*2)...
            + phi_f2*(inc/3-t_d)...
            + phi_f1*(t_d) + (phi_f2 - phi_f1)*(t_d)^3/(t_d^2) - (phi_f2 - phi_f1)*(t_d)^4/((t_d^3)*2)...
            + phi_f1*(inc/3-t_d)...
            + phi_i*t_d + (phi_f1 - phi_i)*(t_d^3)/(t_d^2) - (phi_f1 - phi_i)*(t_d^4)/((t_d^3)*2);

    else

        dphi(ii) = phi_f3;
        ddphi(ii) = 0;
        dphi_int(ii) = phi_f3*(t(ii)-2*inc/3-t_d)...
            + phi_f2*(t_d) + (phi_f3 - phi_f2)*(t_d)^3/(t_d^2) - (phi_f3 - phi_f2)*(t_d)^4/((t_d^3)*2)...
            + phi_f2*(inc/3-t_d)...
            + phi_f1*(t_d) + (phi_f2 - phi_f1)*(t_d)^3/(t_d^2) - (phi_f2 - phi_f1)*(t_d)^4/((t_d^3)*2)...
            + phi_f1*(inc/3-t_d)...
            + phi_i*t_d + (phi_f1 - phi_i)*(t_d^3)/(t_d^2) - (phi_f1 - phi_i)*(t_d^4)/((t_d^3)*2);

    end
    
   Pplot(ii) = kp*(dphi(ii)-y(ii,2)) + kd*(ddphi(ii)-y(ii,4))+ ki*(dphi_int(ii)-y(ii,5));  % PID control of Pressure
if Pplot(ii) < 0
    Pplot(ii) = 0;
elseif Pplot(ii) > Pmax*6894.76
    Pplot(ii) = Pmax*6894.76;
end
    
end

% Calculate trajectory for plotting based on controller (P must be > 0)
phiplot = zeros(1,N);
for jj=1:N
    if (t(jj) <= t_d)
        phiplot(jj) = phi_i + 3*(phi_f1 - phi_i)*(t(jj)^2)/(t_d^2) - 2*(phi_f1 - phi_i)*(t(jj)^3)/(t_d^3);
    elseif (t_d < t(jj)) && (t(jj) <= inc/3)
        phiplot(jj) = phi_f1;
    elseif (inc/3 < t(jj)) && (t(jj) <= inc/3+t_d)   
        phiplot(jj) = phi_f1 + ((t(jj)-inc/3)^2)*3*(phi_f2 - phi_f1)/(t_d^2) - ((t(jj)-inc/3)^3)*2*(phi_f2 - phi_f1)/(t_d^3);
    elseif (inc/3+t_d < t(jj)) && (t(jj) <= 2*inc/3)
        phiplot(jj) = phi_f2;
    elseif (2*inc/3 < t(jj)) && (t(jj) <= 2*inc/3+t_d)
        phiplot(jj) = phi_f2 + ((t(jj)-2*inc/3)^2)*3*(phi_f3 - phi_f2)/(t_d^2) - ((t(jj)-2*inc/3)^3)*2*(phi_f3 - phi_f2)/(t_d^3);
    else    
        phiplot(jj) = phi_f3;
    end
end

% Plot rotation response and trajectory
subplot(1,3,1)
plot(t,y(:,2)*180/pi,'LineWidth',2);
hold on
plot(t,phiplot*180/pi,'LineWidth',2);
xlabel('Time (s)');	
ylabel('Rotation (\circ)');
legend('System response','Polynomial trajectory','Location', 'Best')
grid on; grid minor

% Plot elongation response and trajectory
subplot(1,3,2)
plot(t,y(:,1)*1000,'LineWidth',2);
xlabel('Time (s)');	
ylabel('Elongation (mm)');
grid on; grid minor
 
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
if (t <= t_d)                                   % 1st step starts here
    
    dphi = phi_i + 3*(phi_f1 - phi_i)*(t^2)/(t_d^2) - 2*(phi_f1 - phi_i)*(t^3)/(t_d^3);
    ddphi = 6*(phi_f1 - phi_i)*t/(t_d^2) - 6*(phi_f1 - phi_i)*(t^2)/(t_d^2);
    dphi_int = phi_i*t + (phi_f1 - phi_i)*(t^3)/(t_d^2) - (phi_f1 - phi_i)*(t^4)/((t_d^3)*2);
    
elseif (t_d < t) && (t <= inc/3)      
    
    dphi = phi_f1;
    ddphi = 0;
    dphi_int = phi_f1*(t-t_d)...
        + phi_i*t_d + (phi_f1 - phi_i)*(t_d^3)/(t_d^2) - (phi_f1 - phi_i)*(t_d^4)/((t_d^3)*2);
    
elseif (inc/3 < t) && (t <= inc/3+t_d)          % 2nd step starts here
    
    dphi = phi_f1 + ((t-inc/3)^2)*3*(phi_f2 - phi_f1)/(t_d^2) - ((t-inc/3)^3)*2*(phi_f2 - phi_f1)/(t_d^3);
    ddphi = (t-inc/3)*6*(phi_f2 - phi_f1)/(t_d^2) - ((t-inc/3)^2)*6*(phi_f2 - phi_f1)/(t_d^3);
    dphi_int = phi_f1*(t-inc/3) + (phi_f2 - phi_f1)*(t-inc/3)^3/(t_d^2) - (phi_f2 - phi_f1)*(t-inc/3)^4/((t_d^3)*2)...
        + phi_f1*(inc/3-t_d)...
        + phi_i*t_d + (phi_f1 - phi_i)*(t_d^3)/(t_d^2) - (phi_f1 - phi_i)*(t_d^4)/((t_d^3)*2);

elseif (inc/3+t_d < t) && (t <= 2*inc/3)        
    
     dphi = phi_f2;
     ddphi = 0;
     dphi_int = phi_f2*(t-inc/3-t_d)...
        + phi_f1*(t_d) + (phi_f2 - phi_f1)*(t_d)^3/(t_d^2) - (phi_f2 - phi_f1)*(t_d)^4/((t_d^3)*2)...
        + phi_f1*(inc/3-t_d)...
        + phi_i*t_d + (phi_f1 - phi_i)*(t_d^3)/(t_d^2) - (phi_f1 - phi_i)*(t_d^4)/((t_d^3)*2);

elseif (2*inc/3 < t) && (t <= 2*inc/3+t_d)          % 3rd step starts here
    
    dphi = phi_f2 + ((t-2*inc/3)^2)*3*(phi_f3 - phi_f2)/(t_d^2) - ((t-2*inc/3)^3)*2*(phi_f3 - phi_f2)/(t_d^3);
    ddphi = (t-2*inc/3)*6*(phi_f3 - phi_f2)/(t_d^2) - ((t-2*inc/3)^2)*6*(phi_f3 - phi_f2)/(t_d^3);
    dphi_int = phi_f2*(t-2*inc/3) + (phi_f3 - phi_f2)*(t-2*inc/3)^3/(t_d^2) - (phi_f3 - phi_f2)*(t-2*inc/3)^4/((t_d^3)*2)...
        + phi_f2*(inc/3-t_d)...
        + phi_f1*(t_d) + (phi_f2 - phi_f1)*(t_d)^3/(t_d^2) - (phi_f2 - phi_f1)*(t_d)^4/((t_d^3)*2)...
        + phi_f1*(inc/3-t_d)...
        + phi_i*t_d + (phi_f1 - phi_i)*(t_d^3)/(t_d^2) - (phi_f1 - phi_i)*(t_d^4)/((t_d^3)*2);
    
else
    
    dphi = phi_f3;
    ddphi= 0;
    dphi_int = phi_f3*(t-2*inc/3-t_d)...
        + phi_f2*(t_d) + (phi_f3 - phi_f2)*(t_d)^3/(t_d^2) - (phi_f3 - phi_f2)*(t_d)^4/((t_d^3)*2)...
        + phi_f2*(inc/3-t_d)...
        + phi_f1*(t_d) + (phi_f2 - phi_f1)*(t_d)^3/(t_d^2) - (phi_f2 - phi_f1)*(t_d)^4/((t_d^3)*2)...
        + phi_f1*(inc/3-t_d)...
        + phi_i*t_d + (phi_f1 - phi_i)*(t_d^3)/(t_d^2) - (phi_f1 - phi_i)*(t_d^4)/((t_d^3)*2);
    
end

% dphi = a0 + a2*t^2 + a3*t^3;
% ddphi = 2*a2*t + 3*a3*t^2;
% dphi_int = a0*t + a2*t^3/3 + a3*t^4/4;

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
