function plot_segmented_trajectories(pose_trajectories_kettle,pose_trajectories_bottle,kettle,bottle,table,cup)
    
    colors = [0,0,1;
        0,1,0;
        1,0,1;
        1,0,0;
        1,1,0;
        1,1,1];
    colors = [colors;colors/2;colors/3;colors/4;colors/5;colors/6;colors/7;colors/8;colors/9;colors/10];

    %% case kettle
    figure()
    pose_trajectory = pose_trajectories_kettle(1).pose_trajectory;
    N = size(pose_trajectory,3);
    for k = [1,1+round(0.7*N/4),1+round(1.05*N/4),1+round(2*N/4),N]
        new_hom_vertices = pose_trajectory(:,:,k)*kettle.homogeneous_vertices;
        patch('vertices',new_hom_vertices(1:3,:)','faces',kettle.faces,'facecolor',kettle.color,'edgecolor','none','FaceAlpha',0.1)
        hold on
    end

    pose_trajectory = pose_trajectories_kettle(1).pose_trajectory;
    position_trajectory = squeeze(pose_trajectory(1:3,4,:));
    associated_cluster_indices = pose_trajectories_kettle(1).associated_cluster_indices;
    hold on
    for i = 1:N-3
        if not(isnan(associated_cluster_indices(i)))
            color = colors(associated_cluster_indices(i),:);
        else
            color = [0,0,0];
        end
        scatter3(position_trajectory(1,i+1),position_trajectory(2,i+1),position_trajectory(3,i+1),30,'filled','MarkerFaceColor',color)
        hold on
    end
    
    % Reference point near handle
    pose_trajectory = pose_trajectories_kettle(2).pose_trajectory;
    position_trajectory = squeeze(pose_trajectory(1:3,4,:));
    associated_cluster_indices = pose_trajectories_kettle(2).associated_cluster_indices;
    hold on
    for i = 1:N-3
        if not(isnan(associated_cluster_indices(i)))
            color = colors(associated_cluster_indices(i),:);
        else
            color = [0,0,0];
        end
        scatter3(position_trajectory(1,i+1),position_trajectory(2,i+1),position_trajectory(3,i+1),30,'filled','MarkerFaceColor',color)
        hold on
    end
    hold on

    % Reference point near spout
    pose_trajectory = pose_trajectories_kettle(3).pose_trajectory;
    position_trajectory = squeeze(pose_trajectory(1:3,4,:));
    associated_cluster_indices = pose_trajectories_kettle(3).associated_cluster_indices;
    hold on
    for i = 1:N-3
        if not(isnan(associated_cluster_indices(i)))
            color = colors(associated_cluster_indices(i),:);
        else
            color = [0,0,0];
        end
        scatter3(position_trajectory(1,i+1),position_trajectory(2,i+1),position_trajectory(3,i+1),30,'filled','MarkerFaceColor',color)
        hold on
    end

%     % plot table
%     patch('vertices',table.homogeneous_vertices(1:3,:)','faces',table.faces,'facecolor',table.color,'facealpha',0.6,'edgecolor','none')

    % plot cup
    patch('vertices',(cup.homogeneous_vertices(1:3,:)-[0.005;0.005;0])','faces',cup.faces,'facecolor',cup.color,'facealpha',0.8,'edgecolor','none')

    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
        view(-10,30) % 3D view
%     view(2); % top view
    light
    axis off
%     set(gca,'FontSize',20)


    %% Case bottle
    figure()
    pose_trajectory = pose_trajectories_bottle.pose_trajectory;
    N = size(pose_trajectory,3);
%     for k = 1:round(N/4):N
    for k = [1,1+round(0.7*N/4),1+round(2*N/4),N]
        new_hom_vertices = pose_trajectory(:,:,k)*bottle.homogeneous_vertices;
        patch('vertices',new_hom_vertices(1:3,:)','faces',bottle.faces,'facecolor',bottle.color,'edgecolor','none','FaceAlpha',1)
        hold on
    end
    position_trajectory = squeeze(pose_trajectory(1:3,4,:));
    associated_cluster_indices = pose_trajectories_bottle.associated_cluster_indices;
    hold on
    for i = 1:N-3
        if not(isnan(associated_cluster_indices(i)))
            color = colors(associated_cluster_indices(i),:);
        else
            color = [0,0,0];
        end
        scatter3(position_trajectory(1,i+1),position_trajectory(2,i+1),position_trajectory(3,i+1),30,'filled','MarkerFaceColor',color)
        hold on
    end

    % plot table
    patch('vertices',(table.homogeneous_vertices(1:3,:)-[0;0.1;0])','faces',table.faces,'facecolor',table.color,'facealpha',0.6,'edgecolor','none')

    % plot cup
    patch('vertices',(cup.homogeneous_vertices(1:3,:)+[0.04;-0.02;0])' ,'faces',cup.faces,'facecolor',cup.color,'facealpha',0.8,'edgecolor','none')

    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
    view(-10,30) % 3D view
    light
    axis off
%     set(gca,'FontSize',20)
    
end