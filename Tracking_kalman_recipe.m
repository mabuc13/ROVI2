clear all;
clc;
clear;

dt = 0.0005;
tf = 6;

g = 9.8;

F = [1 0 dt 0;0 1 0 dt;0 0 1 0;0 0 0 1];
G = [0 dt^2/2 0 dt]'; #Discreete differentiation [deltaT² /2 , 0] [0, deltaT² /2], [deltaT, 0], [0, deltaT]  #To calc deltaT use the original FPS of 25 frames per second
H = [1 0 0 0;0 1 0 0]; #4x4 instead of 4x2
u = -g;

ModelNoise   = 0.1; #nok 1
MeasNoise    = 0.4; #Nok 100
R            = MeasNoise^2*eye(size(H,1));
Q            = ModelNoise^2*eye(size(F,1));

Pplus        = eye(4);

QD           = Q*dt;
RD           = R/dt;

% Initialization
x        = [0 0 9 30]';
xhatplus = [0 0 9 30]';

dtPlot  = 0.01;
tPlot   = -inf;

% Initialize arrays for plotting at the end of the program
xArray     = [];
xhatArrayD = [];
tArray     = [];

for t = 0 : dt : tf
    if t >= tPlot + dtPlot
        % Save data for plotting
        tPlot      = t + dtPlot - eps;
        xArray     = [xArray x];
        xhatArrayD = [xhatArrayD xhatplus];
        tArray     = [tArray t];
    end
    % Simulation
    
    y = H * x + [MeasNoise*randn;MeasNoise*randn] / sqrt(dt); #Sæt y lig med det aktuelle koordinat
    % Discrete Kalman filter
    Pmin     = F * Pplus * F' + QD;
    KD       = Pplus * H' * inv(RD);
    xhatmin  = F * xhatplus + G * u; #Estimat før måling
    xhatplus = xhatmin + KD * (y - H*xhatmin); # Resultat koordinater og hastighed
    Pplus    = (eye(4)-KD*H) * Pmin;
end

% Plot data.
close all;
figure; set(gcf,'Color','White');
 
hold on; box on;
plot(xArray(1,:), xArray(2,:), 'b-', 'LineWidth', 2);
plot(xhatArrayD(1,:), xhatArrayD(2,:), 'k:', 'LineWidth', 2)
set(gca,'FontSize',12); 
ylabel('Position (m)');
xlim([0 60]);
xlim([0 60]);
grid on;

