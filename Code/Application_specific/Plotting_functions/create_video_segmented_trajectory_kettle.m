function create_video_segmented_trajectory_kettle(pose_trajectories_kettle,kettle,table,cup)
    
    colors = [0,0,1;
        0,1,0;
        1,0,0;
        1,0,1;
        1,1,0;
        1,1,1];
    colors = [colors;colors/2;colors/3;colors/4;colors/5;colors/6;colors/7;colors/8;colors/9;colors/10];

    fig = figure();
    fig.Color = [1 1 1];
    set(fig, 'Position', get(0, 'Screensize'));
    
    pose_trajectory = pose_trajectories_kettle(1).pose_trajectory;
    N = size(pose_trajectory,3);
    
    counter = 1;    
    for k = 1:1:N-3
        
        clf
        
        pose_trajectory = pose_trajectories_kettle(1).pose_trajectory;
        new_hom_vertices = pose_trajectory(:,:,k+1)*kettle.homogeneous_vertices;
        patch('vertices',new_hom_vertices(1:3,:)','faces',kettle.faces,'facecolor',kettle.color,'edgecolor','none','FaceAlpha',1)
        hold on
        
        % reference point near CoM
        pose_trajectory = pose_trajectories_kettle(1).pose_trajectory;
        position_trajectory = squeeze(pose_trajectory(1:3,4,:));
        associated_cluster_indices = pose_trajectories_kettle(1).associated_cluster_indices;
        hold on
        
        for i = 1:k
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
        for i = 1:k
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
        for i = 1:k
            if not(isnan(associated_cluster_indices(i)))
                color = colors(associated_cluster_indices(i),:);
            else
                color = [0,0,0];
            end
            scatter3(position_trajectory(1,i+1),position_trajectory(2,i+1),position_trajectory(3,i+1),30,'filled','MarkerFaceColor',color)
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
        xlim([-0.7 0.2])
        ylim([-0.3 0.2])
        zlim([-0.1 0.3])
        
        view(-10,20) % 3D view
        light
        set(gca,'FontSize',20)
        
        MovieFrames(counter) = getframe(fig, [2 2 1535 791]);  
        counter = counter + 1;
        pause(0.01)

    end
    
    name = './././Output/video_segmented_trajectories_kettle';
    
    Writer = VideoWriter(name);
    Writer.FrameRate = 10;

    % Open the VideoWriter object, write the movie and close the file
    open(Writer);
    writeVideo(Writer,MovieFrames);
    close(Writer);

end