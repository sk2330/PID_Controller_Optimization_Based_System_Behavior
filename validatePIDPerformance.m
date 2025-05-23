function validatePIDPerformance(x_opt)
    % Define baseline PID parameters 
    baseline_params = [10, 5, 1]; % Adjust these as needed
    
    % Test baseline
    baseline_metrics = testPIDParameters(baseline_params, 'Baseline');
    
    % Test optimized
    optimized_metrics = testPIDParameters(x_opt, 'Optimized');
    
    % Create comparison table
    metrics = {'IAE', 'ISE', 'Settling Time', 'Overshoot', 'Steady-State Error'};
    baseline_values = [baseline_metrics.iae, baseline_metrics.ise, ...
                      baseline_metrics.settlingTime, baseline_metrics.overshoot, ...
                      baseline_metrics.steadyStateError];
    optimized_values = [optimized_metrics.iae, optimized_metrics.ise, ...
                       optimized_metrics.settlingTime, optimized_metrics.overshoot, ...
                       optimized_metrics.steadyStateError];
    
    % Display comparison
    comparison_table = table(metrics', baseline_values', optimized_values', ...
                          'VariableNames', {'Metric', 'Baseline', 'Optimized'});
    disp(comparison_table);
    
    % Plot comparison
    figure;
    subplot(2,1,1);
    plot(baseline_metrics.time, baseline_metrics.angular_velocity, 'b', ...
         optimized_metrics.time, optimized_metrics.angular_velocity, 'r');
    legend('Baseline', 'Optimized');
    title('Angular Velocity Comparison');
    xlabel('Time (s)');
    ylabel('rad/s');
    grid on;
    
    subplot(2,1,2);
    plot(baseline_metrics.time, baseline_metrics.error, 'b', ...
         optimized_metrics.time, optimized_metrics.error, 'r');
    legend('Baseline', 'Optimized');
    title('Error Comparison');
    xlabel('Time (s)');
    ylabel('Error');
    grid on;
end

function metrics = testPIDParameters(params, label)
    % Set PID parameters
    try
        set_param('PID_Optimization_Model/PID_Controller', 'P', num2str(params(1)));
        set_param('PID_Optimization_Model/PID_Controller', 'I', num2str(params(2)));
        set_param('PID_Optimization_Model/PID_Controller', 'D', num2str(params(3)));
    catch ME
        error('Failed to set PID parameters: %s', ME.message);
    end
    
    % Run simulation
    try
        sim_out = sim('PID_Optimization_Model');
    catch ME
        warning('Simulation failed for %s PID: %s', label, ME.message);
        metrics = struct('iae', NaN, 'ise', NaN, 'settlingTime', NaN, ...
                        'overshoot', NaN, 'steadyStateError', NaN, ...
                        'time', [], 'angular_velocity', [], 'error', []);
        return;
    end
    
    % Extract metrics
    metrics.iae = sim_out.IAE.signals.values(end);
    metrics.ise = sim_out.ISE.signals.values(end);
    metrics.settlingTime = calculateSettlingTime(sim_out.Angular_Velocity.time, sim_out.Angular_Velocity.signals.values);
    metrics.overshoot = calculateOvershoot(sim_out.Angular_Velocity.time, sim_out.Angular_Velocity.signals.values);
    metrics.steadyStateError = calculateSteadyStateError(sim_out.Angular_Velocity.time, sim_out.Angular_Velocity.signals.values);
    
    % Time and signals
    metrics.time = sim_out.Angular_Velocity.time;
    metrics.angular_velocity = sim_out.Angular_Velocity.signals.values;
    metrics.error = sim_out.Error_signal.signals.values;
    
    % Display results
    fprintf('\n%s PID (Kp=%.2f, Ki=%.2f, Kd=%.2f):\n', label, params(1), params(2), params(3));
    fprintf('  IAE: %.4f\n', metrics.iae);
    fprintf('  ISE: %.4f\n', metrics.ise);
    fprintf('  Settling Time: %.4f s\n', metrics.settlingTime);
    fprintf('  Overshoot: %.2f%%\n', metrics.overshoot);
    fprintf('  Steady-State Error: %.4f\n\n', metrics.steadyStateError);
end
