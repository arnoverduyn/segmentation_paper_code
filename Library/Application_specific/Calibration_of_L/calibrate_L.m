function L = calibrate_L(parameters,bools,kettle)
% This function determines a value for L based on a simulated input
% trajectory (translational motion) using the proposed calibration
% procedure explained in the paper. The constrained optimization problem
% for L is solved using the built-in fmincon() function

    % Retrieve synthetic calibration data (segment of a pure translation)
    [pose_trajectory,~,N] = design_translational_trajectory(parameters.std_dev_angle,parameters.std_dev_pos,parameters.dt);

    % Smooth synthetic calibration data with a Kalman smoother
    [smooth_pose_trajectory,smooth_posetwist,~] = ...
        smooth_pose_data(pose_trajectory,parameters);

    %% solve the calibration problem

    % solver options
    options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
    options.FunctionTolerance = 10^(-6);
    options.OptimalityTolerance = 10^(-6);
    options.StepTolerance = 10^(-6);
    options.MaxFunctionEvaluations = 10^6;
    options.Display= 'off';

    L = 0.001; % initial value

    % set the constraints
    A = 0; b = 0; Aeq = 0; beq = 0; ub = 10; lb = 0;
    nonlcon = @(L)calculate_nonlinear_inequality_constraint(L,smooth_posetwist',parameters.dt,parameters);

    % define the objective
    f = @(L)maximize_L(L); 

    % solve the problem
    [L,~] = fmincon(f,L,A,b,Aeq,beq,lb,ub,nonlcon,options);


    %% Show output

    if bools.print_level; disp(strcat(['Calibrated value for L: ' num2str(round(L,2)) ' [m]'])); end

    if bools.bool_plot_all_figures
        figure()
        position_trajectory = squeeze(smooth_pose_trajectory(1:3,4,:));
        plot3(position_trajectory(1,:),position_trajectory(2,:),position_trajectory(3,:),'r','LineWidth',5)
        hold on
        for k = 1:round(N/4):N
            new_hom_vertices = pose_trajectory(:,:,k)*kettle.homogeneous_vertices;
            patch('vertices',new_hom_vertices(1:3,:)','faces',kettle.faces,'facecolor',kettle.color,'edgecolor','none')
            hold on
        end
        axis equal
        xlabel('x')
        ylabel('y')
        zlabel('z')
        view(0,30)
        light
        set(gca,'FontSize',20)
    end

end