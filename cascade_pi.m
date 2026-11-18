clear;
clc;
close all;

%% 1. Converter Parameters
Vin1 = 12;      % Initial input voltage (V)
Vin2 = 10;      % Input voltage after disturbance (V)
L = 1e-3;       % Inductor value (1 mH)
C = 1000e-6;    % Output capacitor value (1000 uF) - Increased to reduce undershoot
R1 = 18;        % Initial load resistance (Ohms)
R2 = 9;         % Load resistance after step change (Ohms)
Vref = 18;      % Target output voltage (V)

% Timing parameters for disturbances
tVin = 0.2;     % Time of input voltage change (s)
tLoad = 0.4;    % Time of load resistance change (s)
tEnd = 0.8;     % Total simulation time (s)

%% 2. Cascade Controller Parameters
% Outer Loop (Voltage Controller) - Operates slower to maintain stability
Kpv = 1;        % Proportional gain for voltage loop
Kiv = 100;      % Integral gain for voltage loop
Imax = 10;      % Maximum allowable inductor current for protection (A)

% Inner Loop (Current Controller) - Operates faster for rapid dynamic response
Kpi = 0.05;     % Proportional gain for current loop
Kii = 10;       % Integral gain for current loop
Dmin = 0.05;    % Minimum duty cycle limit
Dmax = 0.90;    % Maximum duty cycle limit
Tsoft = 0.05;   % Soft-start duration to prevent massive inrush current (s)

%% 3. Initial Conditions
% State variables: [Inductor Current (iL); Output Voltage (V); Voltage Integral; Current Integral]
x0 = [0; 0; 0; 0];

%% 4. Solver Settings
% High precision settings for the ODE45 solver
opts = odeset('RelTol',1e-7, 'AbsTol',1e-9, 'MaxStep',1e-5);

%% 5. Run Simulation
% Solving the differential equations of the cascade model
[t,x] = ode45(@(t,x) cascade_pi_model(...
    t,x,Vin1,Vin2,L,C,R1,R2,Vref,...
    Kpv,Kiv,Kpi,Kii,Dmin,Dmax,Imax,Tsoft,tVin,tLoad),...
    [0 tEnd],x0,opts);

% Extracting state variables from solver output
iL = x(:,1);
V = x(:,2);
int_V = x(:,3);
int_I = x(:,4);

%% 6. Reconstruct Duty Cycle & Reference Current for Plotting
D = zeros(size(t));
Iref_plot = zeros(size(t));

for k = 1:length(t)
    % Determine current input voltage based on time
    if t(k) < tVin
        Vin = Vin1; 
    else
        Vin = Vin2; 
    end
    
    % Implement soft-start reference tracking
    Vref_cmd = Vref*min(t(k)/Tsoft,1);
    
    % --- Outer Loop (Voltage to Current Reference) ---
    err_V = Vref_cmd - V(k);
    Iref_unsat = Kpv*err_V + Kiv*int_V(k);
    Iref = max(0, min(Imax, Iref_unsat)); % Saturate current reference between 0 and Imax
    Iref_plot(k) = Iref;
    
    % --- Inner Loop (Current to Duty Cycle) ---
    err_I = Iref - iL(k);
    Dff = Vref_cmd/(Vin+Vref_cmd); % Feedforward term based on steady-state Buck-Boost equation
    D_unsat = Dff + Kpi*err_I + Kii*int_I(k);
    D(k) = max(Dmin,min(Dmax,D_unsat)); % Saturate duty cycle within physical limits
end

%% 7. Plot Output Voltage
figure;
plot(t,V,'LineWidth',1.5);
hold on;
yline(Vref,'--','Reference Voltage');
xline(tVin,':','Input Change');
xline(tLoad,':','Load Change');
grid on;
xlabel('Time (s)');
ylabel('Output Voltage Magnitude (V)');
title('Cascade PI Controlled Converter - Output Voltage');

%% 8. Plot Inductor Current
figure;
plot(t,iL,'b','LineWidth',1.5); hold on;
plot(t,Iref_plot,'--r','LineWidth',1.2);
xline(tVin,':','Input Change');
xline(tLoad,':','Load Change');
grid on;
legend('Inductor Current (I_L)', 'Reference Current (I_{ref})', 'Location', 'Best');
xlabel('Time (s)');
ylabel('Current (A)');
title('Cascade PI Controlled Converter - Inductor Current');

%% 9. Plot Duty Cycle
figure;
plot(t,D,'LineWidth',1.5);
hold on;
yline(Dmin,'--');
yline(Dmax,'--');
grid on;
xlabel('Time (s)');
ylabel('Duty Cycle');
title('Cascade PI Controller - Duty Cycle');

%% 10. Performance Analysis
% Calculate steady-state and transient metrics
initial_region = t >= 0.15 & t < tVin;
V_initial = mean(V(initial_region));

input_region = t >= tVin & t < tLoad;
V_input_min = min(V(input_region));
V_input_max = max(V(input_region));

load_region = t >= tLoad;
V_load_min = min(V(load_region));
V_load_max = max(V(load_region));

V_final = V(end);
iL_final = iL(end);

input_drop = V_initial - V_input_min;
load_overshoot = max(0,(V_load_max-Vref)/Vref*100);
load_undershoot = max(0,(Vref-V_load_min)/Vref*100);
final_error = Vref-V_final;

%% 11. Display Results
fprintf('\n====================================\n');
fprintf(' CASCADE PI CONTROLLED CONVERTER RESULTS\n');
fprintf('====================================\n');
fprintf('Voltage Loop (Outer): Kp = %.4f, Ki = %.4f\n', Kpv, Kiv);
fprintf('Current Loop (Inner): Kp = %.4f, Ki = %.4f\n', Kpi, Kii);
fprintf('Initial Output Voltage = %.4f V\n', V_initial);
fprintf('Minimum after Input Change = %.4f V\n', V_input_min);
fprintf('Maximum after Input Change = %.4f V\n', V_input_max);
fprintf('Input Voltage Drop = %.4f V\n', input_drop);
fprintf('Minimum after Load Change = %.4f V\n', V_load_min);
fprintf('Maximum after Load Change = %.4f V\n', V_load_max);
fprintf('Load Overshoot = %.4f %%\n', load_overshoot);
fprintf('Load Undershoot = %.4f %%\n', load_undershoot);
fprintf('Final Output Voltage = %.4f V\n', V_final);
fprintf('Final Inductor Current = %.4f A\n', iL_final);
fprintf('Final Duty Cycle = %.4f\n', D(end));
fprintf('Final Voltage Error = %.4f V\n', final_error);
fprintf('Maximum Duty Cycle = %.4f\n', max(D));
fprintf('Minimum Duty Cycle = %.4f\n', min(D));
fprintf('====================================\n');

%% 12. Cascade Model Function
function dx = cascade_pi_model(t,x,Vin1,Vin2,L,C,R1,R2,Vref,...
    Kpv,Kiv,Kpi,Kii,Dmin,Dmax,Imax,Tsoft,tVin,tLoad)
    
    % State variables assignments
    iL = x(1);
    V = x(2);
    int_V = x(3);
    int_I = x(4);
    
    % Step disturbance logic
    if t < tVin, Vin = Vin1; else, Vin = Vin2; end
    if t < tLoad, R = R1; else, R = R2; end
    
    % Soft-start logic
    Vref_cmd = Vref*min(t/Tsoft,1);
    
    % --- Outer Voltage Loop ---
    err_V = Vref_cmd - V; % Voltage error calculation
    Iref_unsat = Kpv*err_V + Kiv*int_V; % PI control output (Current reference)
    Iref = max(0, min(Imax, Iref_unsat)); % Saturation limit for protection
    
    % Anti-windup logic for Voltage Loop: Stop integrating if saturated
    if (Iref_unsat > Imax && err_V > 0) || (Iref_unsat < 0 && err_V < 0)
        dint_V = 0;
    else
        dint_V = err_V;
    end
    
    % --- Inner Current Loop ---
    err_I = Iref - iL; % Current error calculation
    Dff = Vref_cmd/(Vin+Vref_cmd); % Feedforward term to aid dynamic response
    D_unsat = Dff + Kpi*err_I + Kii*int_I; % PI control output (Duty cycle)
    D = max(Dmin, min(Dmax, D_unsat)); % Duty cycle saturation
    
    % Anti-windup logic for Current Loop: Stop integrating if duty cycle saturated
    if (D_unsat > Dmax && err_I > 0) || (D_unsat < Dmin && err_I < 0)
        dint_I = 0;
    else
        dint_I = err_I;
    end
    
    % --- Average Plant Equations (Buck-Boost Converter) ---
    diL = (D*Vin - (1-D)*V)/L;  % Inductor current dynamics
    dV = ((1-D)*iL - V/R)/C;    % Capacitor voltage dynamics
    
    % State derivatives vector
    dx = [diL; dV; dint_V; dint_I];
end