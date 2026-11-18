
clear;
clc;
close all;

%% 1. Converter Parameters

Vin1 = 12;          % Initial input voltage
Vin2 = 10;          % Input voltage after disturbance

L = 1e-3;           % Inductor (H)
C = 220e-6;         % Capacitor (F)

R1 = 18;            % Initial load resistance
R2 = 9;             % Load after disturbance

D = 0.6;            % Fixed duty cycle

tVin = 0.1;         % Input disturbance time
tLoad = 0.2;        % Load disturbance time
tEnd = 0.5;         % Simulation end time

%% 2. Initial Conditions

x0 = [0; 0];

%% 3. Solver Settings

opts = odeset('RelTol',1e-7,...
              'AbsTol',1e-9,...
              'MaxStep',1e-5);

%% 4. Run Simulation

[t,x] = ode45(@(t,x) uncontrolled_model(...
    t,x,Vin1,Vin2,L,C,R1,R2,D,tVin,tLoad),...
    [0 tEnd],x0,opts);

iL = x(:,1);
V = x(:,2);

%% 5. Plot Output Voltage

figure;

plot(t,V,'LineWidth',1.5);
hold on;

yline(18,'--','Reference Voltage');
xline(tVin,':','Input Change');
xline(tLoad,':','Load Change');

grid on;

xlabel('Time (s)');
ylabel('Output Voltage Magnitude (V)');
title('Uncontrolled Converter - Output Voltage');

%% 6. Plot Inductor Current

figure;

plot(t,iL,'LineWidth',1.5);

xline(tVin,':','Input Change');
xline(tLoad,':','Load Change');

grid on;

xlabel('Time (s)');
ylabel('Inductor Current (A)');
title('Uncontrolled Converter - Inductor Current');

%% 7. Transient Response Analysis

% Initial steady-state voltage
initial_region = t >= 0.08 & t < tVin;
V_initial = mean(V(initial_region));

% Response after input disturbance
input_region = t >= tVin & t < tLoad;

V_input_min = min(V(input_region));
V_input_max = max(V(input_region));

% Response after load disturbance
load_region = t >= tLoad;

V_load_min = min(V(load_region));
V_load_max = max(V(load_region));

% Final values
V_final = V(end);
iL_final = iL(end);

%% 8. Calculate Performance Values

input_voltage_drop = V_initial - V_input_min;

load_voltage_drop = 18 - V_load_min;

final_voltage_error = 18 - V_final;

%% 9. Display Results

fprintf('\n====================================\n');
fprintf(' UNCONTROLLED CONVERTER RESULTS\n');
fprintf('====================================\n');

fprintf('Initial Output Voltage = %.4f V\n',V_initial);

fprintf('Minimum Voltage after Input Change = %.4f V\n',...
    V_input_min);

fprintf('Maximum Voltage after Input Change = %.4f V\n',...
    V_input_max);

fprintf('Minimum Voltage after Load Change = %.4f V\n',...
    V_load_min);

fprintf('Maximum Voltage after Load Change = %.4f V\n',...
    V_load_max);

fprintf('Final Output Voltage = %.4f V\n',V_final);

fprintf('Final Inductor Current = %.4f A\n',iL_final);

fprintf('Fixed Duty Cycle = %.4f\n',D);

fprintf('Input Disturbance Voltage Drop = %.4f V\n',...
    input_voltage_drop);

fprintf('Load Disturbance Voltage Drop from 18V = %.4f V\n',...
    load_voltage_drop);

fprintf('Final Voltage Error from 18V = %.4f V\n',...
    final_voltage_error);

fprintf('====================================\n');

%% 10. Converter Mathematical Model

function dx = uncontrolled_model(...
    t,x,Vin1,Vin2,L,C,R1,R2,D,tVin,tLoad)

    iL = x(1);
    V = x(2);

    % Input voltage disturbance
    if t < tVin
        Vin = Vin1;
    else
        Vin = Vin2;
    end

    % Load disturbance
    if t < tLoad
        R = R1;
    else
        R = R2;
    end

    % Converter differential equations
    diL = (D*Vin - (1-D)*V)/L;

    dV = ((1-D)*iL - V/R)/C;

    dx = [diL; dV];

end
