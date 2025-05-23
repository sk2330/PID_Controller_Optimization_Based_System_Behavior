function optimizePID(~,~)
    % Number of PID parameters
    nvars = 3;

    % Lower and upper bounds for Kp, Ki, Kd
    lb = [0, 0, 0];
    ub = [100, 100, 10];

    % PSO 
    options = optimoptions('particleswarm', ...
        'SwarmSize', 50, ...
        'MaxIterations', 3, ...
        'Display', 'iter');

    % Run PSO optimization
    [x_opt, fval] = particleswarm(@evaluatePID, nvars, lb, ub, options);
    assignin('base', 'x_opt', x_opt);

    % Display results
    fprintf('\n PSO Optimization Complete:\n');
    fprintf('Optimal PID Parameters: Kp = %.4f, Ki = %.4f, Kd = %.4f\n', x_opt);
    fprintf('Minimum Cost Function Value: %.4f\n\n', fval);

    % Evaluate final optimized response
    evaluatePID(x_opt, true, 'PSO_Optimized');
end

function cost = evaluatePID(params, plotResponse, method)
    % Handle optional arguments
    if nargin < 2, plotResponse = false; end
    if nargin < 3, method = 'Optimization'; end

    % Set PID parameters in Simulink model
    set_param('PID_Optimization_Model/PID_Controller', 'P', num2str(params(1), '%.4f'));
    set_param('PID_Optimization_Model/PID_Controller', 'I', num2str(params(2), '%.4f'));
    set_param('PID_Optimization_Model/PID_Controller', 'D', num2str(params(3), '%.4f'));

    % Configure simulation using modern syntax
    simIn = Simulink.SimulationInput('PID_Optimization_Model');
    simIn = simIn.setVariable('SrcWorkspace', 'current');  % Use function workspace

    % Run simulation
    simOut = sim(simIn);

    % Extract performance metrics
    iae = simOut.get('IAE').signals.values(end);
    ise = simOut.get('ISE').signals.values(end);
    
    % Calculate weighted cost function
    cost = iae + 0.1 * ise;

    % Plot results if requested
    if plotResponse
        figure('Name', ['PID Response - ' method], 'NumberTitle', 'off');
        
        % System response
        subplot(2,1,1);
        plot(simOut.tout, simOut.get('Angular_Velocity').signals.values);
        title(['System Output - ' method]);
        xlabel('Time (s)');
        ylabel('Angular Velocity (rad/s)');
        grid on;
        
        % Control signal
        subplot(2,1,2);
        plot(simOut.tout, simOut.get('Torque').signals.values);
        title('Torque_Control_Signal');
        xlabel('Time (s)');
        ylabel('Torque (N·m)');
        grid on;
    end
end

