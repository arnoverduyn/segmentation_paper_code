function plot_trajectories(pose_trajectories_kettle,pose_trajectories_bottle,kettle,bottle,table,cup,title_name)

    %% case kettle
    figure()
    pose_trajectory = pose_trajectories_kettle(1).pose_trajectory;
    N = size(pose_trajectory,3);
    for k = [1,1+round(0.9*N/4),1+round(1.25*N/4),1+round(2*N/4),N]
        new_hom_vertices = pose_trajectory(:,:,k)*kettle.homogeneous_vertices;
        patch('vertices',new_hom_vertices(1:3,:)','faces',kettle.faces,'facecolor',kettle.color,'edgecolor','none','FaceAlpha',1)
        hold on
    end

    pose_trajectory = pose_trajectories_kettle(1).pose_trajectory;
    position_trajectory = squeeze(pose_trajectory(1:3,4,:));
    plot3(position_trajectory(1,:),position_trajectory(2,:),position_trajectory(3,:),'r','LineWidth',2)
    hold on

    % Reference point near handle
    pose_trajectory = pose_trajectories_kettle(2).pose_trajectory;
    position_trajectory = squeeze(pose_trajectory(1:3,4,:));
    plot3(position_trajectory(1,:),position_trajectory(2,:),position_trajectory(3,:),'g','LineWidth',2)
    hold on

    % Reference point near spout
    pose_trajectory = pose_trajectories_kettle(3).pose_trajectory;
    position_trajectory = squeeze(pose_trajectory(1:3,4,:));
    plot3(position_trajectory(1,:),position_trajectory(2,:),position_trajectory(3,:),'b','LineWidth',2)

    % plot table
    patch('vertices',table.homogeneous_vertices(1:3,:)','faces',table.faces,'facecolor',table.color,'facealpha',0.6,'edgecolor','none')

    % plot cup
    patch('vertices',(cup.homogeneous_vertices(1:3,:)-[0.005;0.005;0])','faces',cup.faces,'facecolor',cup.color,'facealpha',0.8,'edgecolor','none')

    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
%     view(-10,30) % 3D view
    view(2); % top view
    light
    axis off
    title(title_name)
%     set(gca,'FontSize',20)


    %% Case bottle
    figure()
    pose_trajectory = pose_trajectories_bottle.pose_trajectory;
    N = size(pose_trajectory,3);
    for k = 1:round(N/4):N
        new_hom_vertices = pose_trajectory(:,:,k)*bottle.homogeneous_vertices;
        patch('vertices',new_hom_vertices(1:3,:)','faces',bottle.faces,'facecolor',bottle.color,'edgecolor','none','FaceAlpha',1)
        hold on
    end
    position_trajectory = squeeze(pose_trajectory(1:3,4,:));
    plot3(position_trajectory(1,:),position_trajectory(2,:),position_trajectory(3,:),'r','LineWidth',2)
    hold on

    % plot table
    patch('vertices',(table.homogeneous_vertices(1:3,:)-[0;0.1;0])','faces',table.faces,'facecolor',table.color,'facealpha',0.6,'edgecolor','none')

    % plot cup
    patch('vertices',(cup.homogeneous_vertices(1:3,:)+[0.04;-0.02;0])' ,'faces',cup.faces,'facecolor',cup.color,'facealpha',0.8,'edgecolor','none')

    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
    view(0,30) % 3D view
    light
    set(gca,'FontSize',20)
    title(title_name)
    
end