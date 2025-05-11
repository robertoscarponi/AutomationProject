clear all
clc

%% Load all experimental data
fprintf('Starting data loading process from external files...\n');
   
    % Step signals - containing unit step response data
    load('../data/step_signals/steps.mat');

    % Square wave signals - containing periodic square input response data
    load('../data/square_wave_signals/square.mat');

    % Ramp signals - containing various slope ramp input data
    load('../data/ramp_signals/ramp1.mat');
    load('../data/ramp_signals/ramp2.mat');
    load('../data/ramp_signals/ramp3.mat');

    % Sinusoidal signals - containing various frequencies and amplitudes
    load('../data/sinusoidal_signals/sine1.mat');
    load('../data/sinusoidal_signals/sine2.mat');
    load('../data/sinusoidal_signals/sine3.mat');
    load('../data/sinusoidal_signals/sine4.mat');
    load('../data/sinusoidal_signals/sine5.mat');
    load('../data/sinusoidal_signals/sine6.mat');

    % Special sinusoidal signals - with different offset and amplitude characteristics
    load('../data/sinusoidal_signals/sine_largeamplitude_nooffset.mat');
    load('../data/sinusoidal_signals/sine_nooffset.mat');
    load('../data/sinusoidal_signals/sine_offset2.mat');
    
    fprintf('--> Data loading complete. All files successfully imported into workspace.\n');
    
%% Extract variables from data matrices
fprintf('Extracting time, input, and output signals from data matrices...\n');

    
    % Step data extraction (time vector, input, position, output)
    t_vec_step = data032(1,:);
    u_step = data032(2,:);
    x_step = data032(3,:);
    y_step = data032(4,:);
    
    % Ramp data extraction
    % Ramp 1
    t_vec_ramp1 = data001(1,:);
    u_ramp1 = data001(2,:);
    x_ramp1 = data001(3,:);
    y_ramp1 = data001(4,:);
    % Ramp 2
    t_vec_ramp2 = data009(1,:);
    u_ramp2 = data009(2,:);
    x_ramp2 = data009(3,:);
    y_ramp2 = data009(4,:);
    % Ramp 3
    t_vec_ramp3 = data010(1,:);
    u_ramp3 = data010(2,:);
    x_ramp3 = data010(3,:);
    y_ramp3 = data010(4,:);

    % Sinusoidal data extraction
    % Sine 1
    t_vec_sin1 = data002(1,:);
    u_sin1= data002(2,:);
    x_sin1 = data002(3,:);
    y_sin1 = data002(4,:);
    % Sine 2
    t_vec_sin2 = data003(1,:);
    u_sin2 = data003(2,:);
    x_sin2 = data003(3,:);
    y_sin2 = data003(4,:);
    % Sine 3
    t_vec_sin3 = data004(1,:);
    u_sin3 = data004(2,:);
    x_sin3 = data004(3,:);
    y_sin3 = data004(4,:);
    % Sine 4
    t_vec_sin4 = data005(1,:);
    u_sin4 = data005(2,:);
    x_sin4 = data005(3,:);
    y_sin4 = data005(4,:);
    % Sine 5
    t_vec_sin5 = data006(1,:);
    u_sin5 = data006(2,:);
    x_sin5 = data006(3,:);
    y_sin5 = data006(4,:);
    % Sine 6
    t_vec_sin6 = data007(1,:);
    u_sin6 = data007(2,:);
    x_sin6 = data007(3,:);
    y_sin6 = data007(4,:);
    % Sine offset
    t_vec_sin_off = data042(1,:);
    u_sin_off = data042(2,:);
    x_sin_off = data042(3,:);
    y_sin_off = data042(4,:);
    % Sine offset
    t_vec_sin_noOff = data038(1,:);
    u_sin_noOff = data038(2,:);
    x_sin_noOff = data038(3,:);
    y_sin_noOff = data038(4,:);
    % Sine with large amplitude and no offset
    t_vec_sinLarge = data044(1,:);
    u_sinLarge = data044(2,:);
    x_sinLarge = data044(3,:);
    y_sinLarge = data044(4,:);

    % Square wave data extraction
    t_vec_square = data011(1,:);
    u_square = data011(2,:);
    x_square = data011(3,:);
    y_square = data011(4,:);

fprintf('--> Data extraction complete. All signals separated into individual variables.\n');


%% Plot input and output responses for all signals
fprintf('Generating visualization plots for all input-output pairs...\n');
    
    % Step signal plots
    figure
    subplot(2,1,1);
    plotInput(t_vec_step, u_step,...
        "Step input over time", "t [s]", "u(t) [V]");
    subplot(2,1,2);
    plotResponse(t_vec_step, y_step,...
        "Step response over time", "t [s]", "y(t) count/s")
    
    % Ramp signal plots
    % Ramp 1
    figure
    subplot(2,1,1);
    plotInput(t_vec_ramp1, u_ramp1,...
        "Ramp 1 input over time", "t [s]", "u(t) [V]");
    subplot(2,1,2);
    plotResponse(t_vec_ramp1, y_ramp1,...
        "Ramp 1 response over time", "t [s]", "y(t) count/s")
    % Ramp 2
    figure
    subplot(2,1,1);
    plotInput(t_vec_ramp2, u_ramp2,...
        "Ramp 2 input over time", "t [s]", "u(t) [V]");
    subplot(2,1,2);
    plotResponse(t_vec_ramp2, y_ramp2,...
        "Ramp 2 response over time", "t [s]", "y(t) count/s")
    % Ramp 3
    figure
    subplot(2,1,1);
    plotInput(t_vec_ramp3, u_ramp3,...
        "Ramp 3 input over time", "t [s]", "u(t) [V]");
    subplot(2,1,2);
    plotResponse(t_vec_ramp3, y_ramp3,...
        "Ramp 3 response over time", "t [s]", "y(t) count/s")
    
    % Sinusoidal signal plots
    % Sine 1
    figure
    subplot(2,1,1);
    plotInput(t_vec_sin1, u_sin1,...
        "Sine 1 input over time", "t [s]", "u(t) [V]");
    subplot(2,1,2);
    plotResponse(t_vec_sin1, y_sin1,...
        "Sine 1 response over time", "t [s]", "y(t) count/s")
    % Sine 2
    figure
    subplot(2,1,1);
    plotInput(t_vec_sin2, u_sin2,...
        "Sine 2 input over time", "t [s]", "u(t) [V]");
    subplot(2,1,2);
    plotResponse(t_vec_sin2, y_sin2,...
        "Sine 2 response over time", "t [s]", "y(t) count/s")
    % Sine 3
    figure
    subplot(2,1,1);
    plotInput(t_vec_sin3, u_sin3,...
        "Sine 3 input over time", "t [s]", "u(t) [V]");
    subplot(2,1,2);
    plotResponse(t_vec_sin3, y_sin3,...
        "Sine 3 response over time", "t [s]", "y(t) count/s")
    % Sine 4
    figure
    subplot(2,1,1);
    plotInput(t_vec_sin4, u_sin4,...
        "Sine 4 input over time", "t [s]", "u(t) [V]");
    subplot(2,1,2);
    plotResponse(t_vec_sin4, y_sin4,...
        "Sine 4 response over time", "t [s]", "y(t) count/s")
    % Sine 5
    figure
    subplot(2,1,1);
    plotInput(t_vec_sin5, u_sin5,...
        "Sine 5 input over time", "t [s]", "u(t) [V]");
    subplot(2,1,2);
    plotResponse(t_vec_sin5, y_sin5,...
        "Sine 5 response over time", "t [s]", "y(t) count/s")
    % Sine 6
    figure
    subplot(2,1,1);
    plotInput(t_vec_sin6, u_sin6,...
        "Sine 6 input over time", "t [s]", "u(t) [V]");
    subplot(2,1,2);
    plotResponse(t_vec_sin6, y_sin6,...
        "Sine 6 response over time", "t [s]", "y(t) count/s")
    % Sine with offset
    figure
    subplot(2,1,1);
    plotInput(t_vec_sin_off, u_sin_off,...
        "Sine Offset input over time", "t [s]", "u(t) [V]");
    subplot(2,1,2);
    plotResponse(t_vec_sin_off, y_sin_off,...
        "Sine Offset response over time","t [s]", "y(t) count/s")
    % Sine without offset
    figure
    subplot(2,1,1);
    plotInput(t_vec_sin_noOff, u_sin_noOff,...
        "Sine No Offset input over time", "t [s]", "u(t) [V]");
    subplot(2,1,2);
    plotResponse(t_vec_sin_noOff, y_sin_noOff,...
        "Sine No Offset response over time", "t [s]", "y(t) count/s")
    % Sine with large amplitude, no offset
    figure
    subplot(2,1,1);
    plotInput(t_vec_sinLarge, u_sinLarge,...
        "Sine No Offset Large Amplitude input over time",...
        "t [s]", "u(t) [V]");
    subplot(2,1,2);
    plotResponse(t_vec_sinLarge, y_sinLarge,...
        "Sine No Offset Large Amplitude response over time",...
        "t [s]", "y(t) count/s")
    % Square wave plots
    figure
    subplot(2,1,1);
    plotInput(t_vec_square, u_square,...
        "Square input over time",...
        "t [s]", "u(t) [V]");
    subplot(2,1,2);
    plotResponse(t_vec_square, y_square,...
        "Square response over time",...
        "t [s]", "y(t) count/s")
    
    
%% Initial Transfer Function Estimation using step response only
fprintf('Performing initial transfer function estimation using only step response data...\n');

    % Extract partial step data for initial estimation
    t_vec_stepUnitCut = t_vec_step(1:3101);  % Use limited time window
    u_stepUnitCut = u_step(1:3101);          % Input signal portion
    y_stepUnitCut = y_step(1:3101);          % Output signal portion
    tc = 0.002;                             % Sampling time in seconds
    
    % Create iddata object for system identification
    dataStepUnit = iddata(y_stepUnitCut', u_stepUnitCut', tc);
    
    % Estimate transfer function (2nd order numerator, 1st order denominator)
    firstEstimated_tf = tfest(dataStepUnit, 2, 1);
    fprintf('Initial transfer function estimation results:\n');
    present(firstEstimated_tf);

fprintf('--> Initial model estimation complete using step response data only.\n');

%% Validate initial model against full step response
fprintf('Validating initial model against complete step response data...\n');
    % Simulate system response to step input using estimated model
    estimatedStepResponse = lsim(firstEstimated_tf, u_step, t_vec_step);

    % Plot comparison between actual and estimated responses
    validationPlot(t_vec_step,y_step, estimatedStepResponse, ...
        "t [s]", "y_{real}(t) [count/s]", "y_{estimated}(t) [count/s]", "Step Response")


%% Improved transfer function estimation using all available data
fprintf('Performing improved estimation using all available experimental data...\n');

    % Create iddata objects for each experiment
    dataStep = iddata(y_step', u_step', tc);
    dataRamp1 = iddata(y_ramp1', u_ramp1', tc);
    dataRamp2 = iddata(y_ramp2', u_ramp2', tc);
    dataRamp3 = iddata(y_ramp3', u_ramp3', tc);
    dataSin1 = iddata(y_sin1', u_sin1', tc);
    dataSin2 = iddata(y_sin2', u_sin2', tc);
    dataSin3 = iddata(y_sin3', u_sin3', tc);
    dataSin4 = iddata(y_sin4', u_sin4', tc);
    dataSin5 = iddata(y_sin5', u_sin5', tc);
    dataSin6 = iddata(y_sin6', u_sin6', tc);
    dataSinOff = iddata(y_sin_off', u_sin_off', tc);
    dataSinNoOff = iddata(y_sin_noOff', u_sin_noOff', tc);
    dataSinLarge = iddata(y_sinLarge', u_sinLarge', tc);
    dataSquare = iddata(y_square', u_square', tc);

    % Combine all datasets into a comprehensive dataset for improved estimation
    data_combined = merge(dataStep, dataRamp1, dataRamp2, dataRamp3,...
        dataSin1, dataSin2, dataSin3, dataSin4, dataSin5,...
        dataSin6,dataSinOff, dataSinNoOff,dataSinLarge, dataSquare);

    % Estimate improved transfer function using the comprehensive dataset
    newEstimated_tf = tfest(data_combined, 2, 1);

fprintf('--> Improved transfer function estimated using comprehensive dataset.\n');

%% Validate improved model against various inputs
fprintf('Validating improved model against multiple input types...\n');
    
    % Display the transfer function details
    present(newEstimated_tf);
    
    % Validate against step response
    newEstimatedStepResponse = lsim(newEstimated_tf, u_step, t_vec_step);
    validationPlot(t_vec_step,y_step, newEstimatedStepResponse, ...
        "t [s]", "y_{real}(t) [count/s]", "y_{estimated}(t) [count/s]", "Step Response")

    % Validate against more challenging input signals
    % Sinusoidal with offset
    newEstimatedSinOffResponse = lsim(newEstimated_tf, u_sin_off, t_vec_sin_off);
    validationPlot(t_vec_sin_off, y_sin_off, newEstimatedSinOffResponse, ...
        "t [s]", "y_{real}(t) [count/s]", "y_{estimated}(t) [count/s]",...
        "Sine with Offset Response");

    % Sinusoidal without offset
    newEstimatedSin_NoOffResponse = lsim(newEstimated_tf, u_sin_noOff, t_vec_sin_noOff);
    validationPlot(t_vec_sin_off, y_sin_off, newEstimatedSinOffResponse, ...
        "t [s]", "y_{real}(t) [count/s]", "y_{estimated}(t) [count/s]",...
        "Sine without Offset Response");

    % Square wave (most challenging for linear models)
    newEstimatedSquareResponse = lsim(newEstimated_tf, u_square, t_vec_square);
    validationPlot(t_vec_square, y_square, newEstimatedSquareResponse, ...
        "t [s]", "y_{real}(t) [count/s]", "y_{estimated}(t) [count/s]",...
        "Square Wave Response");

%% Final model optimization with custom dataset weighting
fprintf('Optimizing final model with emphasis on challenging input patterns...\n');

   
    % Create a weighted dataset with multiple instances of challenging signals
    % to bias the estimation toward better fitting these difficult cases
    optimizated_data_combined = merge(dataStep, dataRamp1, dataRamp2, dataRamp3,...
        dataSin1, dataSin2, dataSin3, dataSin4, dataSin5,...
        dataSin5, dataSin6,dataSin6, dataSinOff, dataSinOff, dataSinOff,...
        dataSinNoOff,dataSinNoOff, dataSinNoOff, dataSinLarge, dataSquare);

    % Estimate final optimized transfer function
    finalEstimated_tf = tfest(optimizated_data_combined, 2, 1);

fprintf('--> Final optimized transfer function estimated with custom weighting.\n');


%% Comprehensive validation of final model against all input types
fprintf('Performing comprehensive validation of final model across all input patterns...\n');
    
    
    % Simulate and validate responses for all input types
    % Step response
    stepResponse = lsim(finalEstimated_tf, u_step, t_vec_step);
    validationPlot(t_vec_step, y_step, stepResponse, ...
        "t [s]", "y_{real}(t) [count/s]", "y_{estimated}(t) [count/s]",...
        "Step Response");
    
    % Square wave response
    squareResponse = lsim(finalEstimated_tf, u_square, t_vec_square);
    validationPlot(t_vec_square, y_square, squareResponse, ...
        "t [s]", "y_{real}(t) [count/s]", "y_{estimated}(t) [count/s]",...
        "Square Wave Response");
    
    % Ramp 1 response
    ramp1Response = lsim(finalEstimated_tf, u_ramp1, t_vec_ramp1);
    validationPlot(t_vec_ramp1, y_ramp1, ramp1Response, ...
        "t [s]", "y_{real}(t) [count/s]", "y_{estimated}(t) [count/s]",...
        "Ramp 1 Response");
    
    % Ramp 2 response
    ramp2Response = lsim(finalEstimated_tf, u_ramp2, t_vec_ramp2);
    validationPlot(t_vec_ramp2, y_ramp2, ramp2Response, ...
        "t [s]", "y_{real}(t) [count/s]", "y_{estimated}(t) [count/s]",...
        "Ramp 2 Response");
    
    % Ramp 3 response
    ramp3Response = lsim(finalEstimated_tf, u_ramp3, t_vec_ramp3);
    validationPlot(t_vec_ramp3, y_ramp3, ramp3Response, ...
        "t [s]", "y_{real}(t) [count/s]", "y_{estimated}(t) [count/s]",...
        "Ramp 3 Response");
    
    % Sine 1 response
    sin1Response = lsim(finalEstimated_tf, u_sin1, t_vec_sin1);
    validationPlot(t_vec_sin1, y_sin1, sin1Response, ...
        "t [s]", "y_{real}(t) [count/s]", "y_{estimated}(t) [count/s]",...
        "Sine 1 Response");
    
    % Sine 2 response
    sin2Response = lsim(finalEstimated_tf, u_sin2, t_vec_sin2);
    validationPlot(t_vec_sin2, y_sin2, sin2Response, ...
        "t [s]", "y_{real}(t) [count/s]", "y_{estimated}(t) [count/s]",...
        "Sine 2 Response");
    
    % Sine 3 response
    sin3Response = lsim(finalEstimated_tf, u_sin3, t_vec_sin3);
    validationPlot(t_vec_sin3, y_sin3, sin3Response, ...
        "t [s]", "y_{real}(t) [count/s]", "y_{estimated}(t) [count/s]",...
        "Sine 3 Response");
    
    % Sine 4 response
    sin4Response = lsim(finalEstimated_tf, u_sin4, t_vec_sin4);
    validationPlot(t_vec_sin4, y_sin4, sin4Response, ...
        "t [s]", "y_{real}(t) [count/s]", "y_{estimated}(t) [count/s]",...
        "Sine 4 Response");
    
    % Sine 5 response
    sin5Response = lsim(finalEstimated_tf, u_sin5, t_vec_sin5);
    validationPlot(t_vec_sin5, y_sin5, sin5Response, ...
        "t [s]", "y_{real}(t) [count/s]", "y_{estimated}(t) [count/s]",...
        "Sine 5 Response");
    
    % Sine 6 response
    sin6Response = lsim(finalEstimated_tf, u_sin6, t_vec_sin6);
    validationPlot(t_vec_sin6, y_sin6, sin6Response, ...
        "t [s]", "y_{real}(t) [count/s]", "y_{estimated}(t) [count/s]",...
        "Sine 6 Response");
    
    % Sine with offset response
    sinOffsetResponse = lsim(finalEstimated_tf, u_sin_off, t_vec_sin_off);
    validationPlot(t_vec_sin_off, y_sin_off, sinOffsetResponse, ...
        "t [s]", "y_{real}(t) [count/s]", "y_{estimated}(t) [count/s]",...
        "Sine with Offset Response");
    
    % Sine without offset response
    sinNoOffsetResponse = lsim(finalEstimated_tf, u_sin_noOff, t_vec_sin_noOff);
    validationPlot(t_vec_sin_noOff, y_sin_noOff, sinNoOffsetResponse, ...
        "t [s]", "y_{real}(t) [count/s]", "y_{estimated}(t) [count/s]",...
        "Sine without Offset Response");
    
    % Sine with large amplitude response
    sinNoOffsetLargeResponse = lsim(finalEstimated_tf, u_sinLarge, t_vec_sinLarge);
    validationPlot(t_vec_sinLarge, y_sinLarge, sinNoOffsetLargeResponse, ...
        "t [s]", "y_{real}(t) [count/s]", "y_{estimated}(t) [count/s]",...
        "Sine with Large Amplitude Response");

%% Display final transfer function coefficients
num_tf = finalEstimated_tf.Numerator;
den_tf = finalEstimated_tf.Denominator;
fprintf('Final transfer function coefficients:\n');
fprintf('  Numerator: %s\n', num2str(num_tf));
fprintf('  Denominator: %s\n', num2str(den_tf));

%% Frequency domain analysis with Bode plot
fprintf('Generating frequency response (Bode) plot for final model...\n');
figure
bode(finalEstimated_tf);
title('Bode Plot of Final Estimated Transfer Function');
    
%% State Space representation of the final model
fprintf('Converting transfer function to state space representation...\n');

ss_sys = ss(finalEstimated_tf);
fprintf('State space model of the system:\n');
present(ss_sys);

%% Response Analysis with Different Initial Conditions
fprintf('Analyzing system behavior with non-zero initial conditions...\n');

% Define simulation parameters
t_final = 3;                     % Simulation duration in seconds
dt = 0.002;                      % Time step matching experimental data
t_sim = 0:dt:t_final;            % Time vector for simulation

% Create unit step input signal
u_sim = ones(size(t_sim));       % Unit step input (magnitude = 1)

% Define non-zero initial conditions for state variables
x0_nonzero = [40; 10];           % Initial values for state variables
                                 % State 1: position, State 2: velocity (interpretation depends on state-space form)

% Extract system matrices from state-space representation
A = ss_sys.A;                    % System matrix
B = ss_sys.B;                    % Input matrix
C = ss_sys.C;                    % Output matrix
D = ss_sys.D;                    % Direct transmission matrix

% Define zero initial conditions for comparison
x0_zero = zeros(size(A,1), 1);   % Zero initial state vector

% Simulate total response (forced + natural) with non-zero initial conditions
[y_total, t_out_lsim, x_total] = lsim(ss_sys, u_sim, t_sim, x0_nonzero);

% Simulate forced response only (with zero initial conditions)
[y_forced_step, t_out_step] = step(finalEstimated_tf, t_sim);

% Create comparative visualization of responses
figure;
hold on;
plot(t_out_lsim, y_total, 'Color', [0, 0.4470, 0.7410],...
    'LineWidth', 2.5, 'DisplayName', 'Total Response (Non-zero IC)');
plot(t_out_step, y_forced_step, 'Color', [0.8500, 0.3250, 0.0980],...
    'LineStyle', '--', 'LineWidth', 2.5,...
    'DisplayName', 'Forced Response (Zero IC)');

xlabel('Time (s)');
ylabel('Output y(t)');
title('System Response: Impact of Initial Conditions');
legend('show', 'Location', 'SouthEast');
grid on;
box on;
hold off;


fprintf('Analysis complete. All system identification and validation tasks finished.\n');
