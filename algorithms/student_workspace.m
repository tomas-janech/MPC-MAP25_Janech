function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

% probabilities
cycles = 300;

if read_only_vars.counter < cycles + 1
    public_vars.lidar_history(read_only_vars.counter, :) = read_only_vars.lidar_distances;
end

if read_only_vars.counter == cycles+1
    std_GNSS = std(read_only_vars.gnss_history)

    std_LIDAR = std(public_vars.lidar_history)

end


% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
    
    public_vars.lidar_history = zeros(cycles,8);

    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);

end

% 9. Update particle filter
public_vars.particles = update_particle_filter(read_only_vars, public_vars);

% 10. Update Kalman filter
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)

% 12. Path planning
public_vars.path = plan_path(read_only_vars, public_vars);

% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);



end

