clear;
clc;
close all;

%% 1. Converter Parameters
Vin1 = 12;
Vin2 = 10;
L = 1e-3;
C = 1000e-6;  
R1 = 18;
R2 = 9;
Vref = 18;


tVin = 0.2;    
tLoad = 0.4;  
tEnd = 1.0;     
% ---------------------------------------------------

%% 2. PI Controller Parameters
Kp = 0.004;     
Ki = 0.2;
Dmin = 0.05;
Dmax = 0.90;
Tsoft = 0.05;

%% 3. Initial Conditions
x0 = [0; 0; 0];

%% 4. Solver Settings
opts = odeset('RelTol',1e-7,...
              'AbsTol',1e-9,...
              'MaxStep',1e-5);

%% 5. Run Simulation
[t,x] = ode45(@(t,x) pi_model(...
    t,x,Vin1,Vin2,L,C,R1,R2,Vref,...
    Kp,Ki,Dmin,Dmax,Tsoft,tVin,tLoad),...
    [0 tEnd],x0,opts);

iL = x(:,1);
V = x(:,2);
integral_error = x(:,3);

%% 6. Reconstruct Duty Cycle
D = zeros(size(t));
for k = 1:length(t)
    if t(k) < tVin
        Vin = Vin1;
    else
        Vin = Vin2;
    end
    Vref_cmd = Vref*min(t(k)/Tsoft,1);
    error = Vref_cmd - V(k);
    Dff = Vref_cmd/(Vin+Vref_cmd);
    D_unsat = Dff + Kp*error + Ki*integral_error(k);
    D(k) = max(Dmin,min(Dmax,D_unsat));
end

%% 7. Plot Output Voltage
figure;
plot(t,V,'LineWidth',1.5);
hold on;
yline(Vref,'--','Reference Voltage');
xline(tVin,':','Input Change (0.2s)');
xline(tLoad,':','Load Change (0.4s)');
grid on;
xlabel('Time (s)');
ylabel('Output Voltage Magnitude (V)');
title(sprintf('PI Controlled Converter - Output Voltage (C = %g \\muF)', C*1e6));

%% 8. Plot Inductor Current
figure;
plot(t,iL,'LineWidth',1.5);
xline(tVin,':','Input Change');
xline(tLoad,':','Load Change');
grid on;
xlabel('Time (s)');
ylabel('Inductor Current (A)');
title('PI Controlled Converter - Inductor Current');

%% 9. Plot Duty Cycle
figure;
plot(t,D,'LineWidth',1.5);
hold on;
yline(Dmin,'--');
yline(Dmax,'--');
grid on;
xlabel('Time (s)');
ylabel('Duty Cycle');
title('PI Controller - Duty Cycle');

%% 10. Performance Analysis

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
fprintf(' PI CONTROLLED CONVERTER RESULTS (HIGH C & ADJUSTED TIME)\n');
fprintf('====================================\n');
fprintf('Kp = %.4f\n',Kp);
fprintf('Ki = %.4f\n',Ki);
fprintf('Initial Output Voltage (Before 0.2s) = %.4f V\n',V_initial);
fprintf('Minimum after Input Change = %.4f V\n', V_input_min);
fprintf('Maximum after Input Change = %.4f V\n', V_input_max);
fprintf('Input Voltage Drop = %.4f V\n',input_drop);
fprintf('Minimum after Load Change = %.4f V\n', V_load_min);
fprintf('Maximum after Load Change = %.4f V\n', V_load_max);
fprintf('Load Overshoot = %.4f %%\n',load_overshoot);
fprintf('Load Undershoot = %.4f %%\n',load_undershoot);
fprintf('Final Output Voltage = %.4f V\n',V_final);
fprintf('Final Inductor Current = %.4f A\n',iL_final);
fprintf('Final Duty Cycle = %.4f\n',D(end));
fprintf('Final Voltage Error = %.4f V\n',final_error);
fprintf('====================================\n');

%% 12. PI Controller + Converter Model
function dx = pi_model(...
    t,x,Vin1,Vin2,L,C,R1,R2,Vref,...
    Kp,Ki,Dmin,Dmax,Tsoft,tVin,tLoad)
    
    iL = x(1);
    V = x(2);
    integral_error = x(3);

    if t < tVin
        Vin = Vin1;
    else
        Vin = Vin2;
    end

    if t < tLoad
        R = R1;
    else
        R = R2;
    end

    Vref_cmd = Vref*min(t/Tsoft,1);
    error = Vref_cmd-V;
    Dff = Vref_cmd/(Vin+Vref_cmd);
    
    D_unsat = Dff + Kp*error + Ki*integral_error;
    D = max(Dmin,min(Dmax,D_unsat));
    
    if (D_unsat > Dmax && error > 0) || ...
       (D_unsat < Dmin && error < 0)
        dintegral = 0;
    else
        dintegral = error;
    end

    diL = (D*Vin-(1-D)*V)/L;
    dV = ((1-D)*iL-V/R)/C;
    
    dx = [diL; dV; dintegral];
end
