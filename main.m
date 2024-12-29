% Animated Adaptive Radar Simulation in MATLAB  

% Parameters  
Fs = 1e6;            % Sampling frequency (Hz)  
T = 1e-3;           % Simulation time (s)  
t = 0:1/Fs:T-1/Fs;  % Time vector  
numSamples = length(t); % Number of samples  

% Target parameters  
targetPos = 0.3;     % Target position (relative to time)  
targetAmp = 1;       % Target amplitude  
targetFreq = 2000;   % Target frequency (Hz)  

% Generate target signal  
targetSignal = targetAmp * cos(2 * pi * targetFreq * t);  
% Add noise  
noisePower = 0.1; % Noise power  
noiseSignal = sqrt(noisePower) * randn(1, numSamples);  
receivedSignal = targetSignal + noiseSignal;  

% Adaptive CFAR parameters  
numGuardCells = 2;  % Number of guard cells  
numTrainingCells = 10; % Number of training cells  
windowSize = numGuardCells + numTrainingCells; % Total window size  
thresholdFactor = 1.5; % Dynamic threshold factor  

% Initialize output signal  
outputSignal = zeros(1, numSamples);  

% Prepare figure for animation  
figure;  
hReceived = plot(t, receivedSignal, 'b');  
hold on;  
hOutput = plot(t, outputSignal, 'r', 'LineWidth', 2);  
hThreshold = plot(t, zeros(1, numSamples), 'k--', 'LineWidth', 1);  
axis([0 T -2 2]);  
xlabel('Time (s)');  
ylabel('Amplitude');  
title('Adaptive Radar Animation');  
legend('Received Signal', 'Detection Output', 'Dynamic Threshold');  
grid on;  

% Animation loop  
for i = 1 + windowSize : numSamples - windowSize  
    % Get the training cells  
    trainingCells = receivedSignal(i-windowSize : i+windowSize);  
    % Remove the guard cells  
    trainingCells([1:numGuardCells, end-numGuardCells+1:end]) = [];  
    
    % Calculate the threshold  
    threshold = thresholdFactor * mean(trainingCells.^2);  
    
    % Detect signal  
    if (receivedSignal(i)^2 > threshold)  
        outputSignal(i) = 1; % Detection  
    else  
        outputSignal(i) = 0; % No detection  
    end  
    
    % Update plot for current time step  
    set(hReceived, 'YData', receivedSignal);  
    set(hOutput, 'YData', outputSignal);  
    set(hThreshold, 'YData', threshold * ones(size(t)));  
    
    % Pause for a moment to visualize the animation  
    pause(1/Fs);  
end  

% Final update to synchronize outputs in the last frame  
set(hOutput, 'YData', outputSignal);  
set(hThreshold, 'YData', threshold * ones(size(t)));
