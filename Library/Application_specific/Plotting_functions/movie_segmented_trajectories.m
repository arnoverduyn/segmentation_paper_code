function movie_segmented_trajectories(pose_trajectories_kettle,pose_trajectories_bottle,kettle,bottle,table,cup,samples,pause_time)
    
    colors = [0,0,1;
        0,1,0;
        1,0,0;
        1,0,1;
        1,1,0;
        1,1,1];
    colors = [colors;colors/2;colors/3;colors/4;colors/5;colors/6;colors/7;colors/8;colors/9;colors/10];

    fig = figure();
    set(fig, 'Position', get(0, 'Screensize'));
    pose_trajectory = pose_trajectories_kettle(1).pose_trajectory;
    N = size(pose_trajectory,3);
    
    for k = 2:samples:N-2
        clf
        
        subplot(2,2,1)
        % plot trajectory
        pose_trajectory = pose_trajectories_kettle(1).pose_trajectory;
        new_hom_vertices = pose_trajectory(:,:,k)*kettle.homogeneous_vertices;
        patch('vertices',new_hom_vertices(1:3,:)','faces',kettle.faces,'facecolor',kettle.color,'edgecolor','none','FaceAlpha',1)
        hold on
        position_trajectory = squeeze(pose_trajectory(1:3,4,:));
        associated_cluster_indices = pose_trajectories_kettle(1).associated_cluster_indices;
        hold on
        for i = 2:samples:k
            if not(isnan(associated_cluster_indices(i-1)))
                color = colors(associated_cluster_indices(i-1),:);
            else
                color = [0,0,0];
            end
            scatter3(position_trajectory(1,i),position_trajectory(2,i),position_trajectory(3,i),20,'filled','MarkerFaceColor',color)
            hold on
        end
        
        % plot table
        patch('vertices',table.homogeneous_vertices(1:3,:)','faces',table.faces,'facecolor',table.color,'facealpha',0.6,'edgecolor','none')
        % plot cup
        patch('vertices',(cup.homogeneous_vertices(1:3,:)-[0.005;0.005;0])','faces',cup.faces,'facecolor',cup.color,'facealpha',0.8,'edgecolor','none')

        axis equal
        xlabel('x')
        ylabel('y')
        zlabel('z')
        view(-10,30) % 3D view
        light
        set(gca,'FontSize',20)
        xlim([-0.8 0.3])
        ylim([-0.4 0.3])
        zlim([-0.2 0.4])
        
        subplot(2,2,2)
        % plot trajectory
        pose_trajectory = pose_trajectories_kettle(1).pose_trajectory;
        new_hom_vertices = pose_trajectory(:,:,k)*kettle.homogeneous_vertices;
        patch('vertices',new_hom_vertices(1:3,:)','faces',kettle.faces,'facecolor',kettle.color,'edgecolor','none','FaceAlpha',1)
        hold on
        position_trajectory = squeeze(pose_trajectories_kettle(2).pose_trajectory(1:3,4,:));
        associated_cluster_indices = pose_trajectories_kettle(2).associated_cluster_indices;
        hold on
        for i = 2:samples:k
            if not(isnan(associated_cluster_indices(i-1)))
                color = colors(associated_cluster_indices(i-1),:);
            else
                color = [0,0,0];
            end
            scatter3(position_trajectory(1,i),position_trajectory(2,i),position_trajectory(3,i),20,'filled','MarkerFaceColor',color)
            hold on
        end
        hold on
        % plot table
        patch('vertices',table.homogeneous_vertices(1:3,:)','faces',table.faces,'facecolor',table.color,'facealpha',0.6,'edgecolor','none')
        % plot cup
        patch('vertices',(cup.homogeneous_vertices(1:3,:)-[0.005;0.005;0])','faces',cup.faces,'facecolor',cup.color,'facealpha',0.8,'edgecolor','none')

        axis equal
        xlabel('x')
        ylabel('y')
        zlabel('z')
        view(-10,30) % 3D view
        light
        set(gca,'FontSize',20)
        xlim([-0.8 0.3])
        ylim([-0.4 0.3])
        zlim([-0.2 0.4])
        
        subplot(2,2,3)
        % plot trajectory
        pose_trajectory = pose_trajectories_kettle(1).pose_trajectory;
        new_hom_vertices = pose_trajectory(:,:,k)*kettle.homogeneous_vertices;
        patch('vertices',new_hom_vertices(1:3,:)','faces',kettle.faces,'facecolor',kettle.color,'edgecolor','none','FaceAlpha',1)
        hold on
        position_trajectory = squeeze(pose_trajectories_kettle(3).pose_trajectory(1:3,4,:));
        associated_cluster_indices = pose_trajectories_kettle(3).associated_cluster_indices;
        hold on
        for i = 2:samples:k
            if not(isnan(associated_cluster_indices(i-1)))
                color = colors(associated_cluster_indices(i-1),:);
            else
                color = [0,0,0];
            end
            scatter3(position_trajectory(1,i),position_trajectory(2,i),position_trajectory(3,i),20,'filled','MarkerFaceColor',color)
            hold on
        end
        hold on
        % plot table
        patch('vertices',table.homogeneous_vertices(1:3,:)','faces',table.faces,'facecolor',table.color,'facealpha',0.6,'edgecolor','none')
        % plot cup
        patch('vertices',(cup.homogeneous_vertices(1:3,:)-[0.005;0.005;0])','faces',cup.faces,'facecolor',cup.color,'facealpha',0.8,'edgecolor','none')

        axis equal
        xlabel('x')
        ylabel('y')
        zlabel('z')
        view(-10,30) % 3D view
        light
        set(gca,'FontSize',20)
        xlim([-0.8 0.3])
        ylim([-0.4 0.3])
        zlim([-0.2 0.4])
        
        subplot(2,2,4)
        % plot trajectory
        pose_trajectory = pose_trajectories_bottle.pose_trajectory;
        if k > size(pose_trajectory,3)-2
            index = size(pose_trajectory,3)-2;
        else
            index = k;
        end
        
        new_hom_vertices = pose_trajectory(:,:,index)*bottle.homogeneous_vertices;
        patch('vertices',new_hom_vertices(1:3,:)','faces',bottle.faces,'facecolor',bottle.color,'edgecolor','none','FaceAlpha',1)
        hold on
        position_trajectory = squeeze(pose_trajectories_bottle.pose_trajectory(1:3,4,:));
        associated_cluster_indices = pose_trajectories_bottle.associated_cluster_indices;
        hold on
        for i = 2:samples:index
            if not(isnan(associated_cluster_indices(i-1)))
                color = colors(associated_cluster_indices(i-1),:);
            else
                color = [0,0,0];
            end
            scatter3(position_trajectory(1,i),position_trajectory(2,i),position_trajectory(3,i),20,'filled','MarkerFaceColor',color)
            hold on
        end
        hold on
        % plot table
        patch('vertices',(table.homogeneous_vertices(1:3,:)-[0;0.1;0])','faces',table.faces,'facecolor',table.color,'facealpha',0.6,'edgecolor','none')
        % plot cup
        patch('vertices',(cup.homogeneous_vertices(1:3,:)+[0.03;-0.02;0])' ,'faces',cup.faces,'facecolor',cup.color,'facealpha',0.8,'edgecolor','none')

        axis equal
        xlabel('x')
        ylabel('y')
        zlabel('z')
        view(-10,30) % 3D view
        light
        set(gca,'FontSize',20)
        xlim([-0.8 0.3])
        ylim([-0.4 0.3])
        zlim([-0.2 0.4])
        
        pause(pause_time)

    end
end