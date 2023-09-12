function plot_real_data(pose_trajectories_kettle,kettle,table,cup)

    figure()
    pose_trajectory = pose_trajectories_kettle(1).pose_trajectory;
    N = size(pose_trajectory,3);
    T_delta = [rotz(145),[0;0;0]; 0 0 0 1];
    for k = 1:N
        pose_trajectory(:,:,k) = T_delta*pose_trajectory(:,:,k);
    end

    for k = [1,1+round(0.6*N/4),1+round(0.95*N/4),1+round(2*N/4),1+round(3.62*N/4)]
        vertices = [kettle.homogeneous_vertices(1:3,:)*1;kettle.homogeneous_vertices(4,:)];
        vertices(1:3,:) = vertices(1:3,:) + [-0.05;0.04;-0.01];
        vertices = [rotx(4)*rotz(150)*rotx(180),[0;0;0];0,0,0,1]*vertices; %reorient_kettle
        
        new_hom_vertices = pose_trajectory(:,:,k)*vertices;
        patch('vertices',new_hom_vertices(1:3,:)','faces',kettle.faces,'facecolor',kettle.color,'edgecolor','none','FaceAlpha',1)
        hold on
    end
    
    position_trajectory = squeeze(pose_trajectory(1:3,4,:));
    plot3(position_trajectory(1,:),position_trajectory(2,:),position_trajectory(3,:),'r','LineWidth',3)
    hold on
    
    % plot table
    patch('vertices',table.homogeneous_vertices(1:3,:)'+[-0.9,-1.87,-2.08],'faces',table.faces,'facecolor',table.color,'facealpha',0.6,'edgecolor','none')

    % plot cup
    cup_vertices = 1.3*cup.homogeneous_vertices(1:3,:);
    cup_vertices = cup_vertices - mean(cup_vertices,2);
    patch('vertices',(cup_vertices-[1.4;1.895;2.06])','faces',cup.faces,'facecolor',cup.color,'facealpha',0.8,'edgecolor','none')

    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
    view(0,40) % 3D view
    light
    axis off
end