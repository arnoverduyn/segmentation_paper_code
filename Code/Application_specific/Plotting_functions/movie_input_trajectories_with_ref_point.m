function movie_input_trajectories_with_ref_point(pose_trajectories_kettle,pose_trajectories_bottle,kettle,bottle,table,cup)
    fig = figure();
    set(fig, 'Position', get(0, 'Screensize'));
    pose_trajectory = pose_trajectories_kettle(1).pose_trajectory;
    N = size(pose_trajectory,3);
    
    for k = 1:5:N
        clf
                
        subplot(2,2,1)
        % plot trajectory
        pose_trajectory = pose_trajectories_kettle(1).pose_trajectory;
        new_hom_vertices = pose_trajectory(:,:,k)*kettle.homogeneous_vertices;
        patch('vertices',new_hom_vertices(1:3,:)','faces',kettle.faces,'facecolor',kettle.color,'edgecolor','none','FaceAlpha',1)
        hold on
        p_ref = pose_trajectories_kettle(1).p_ref;
        scatter3(p_ref(1,1:k),p_ref(2,1:k),p_ref(3,1:k),5,'k','filled')
        hold on
        scatter3(p_ref(1,k),p_ref(2,k),p_ref(3,k),100,'k','filled')
        hold on
        position_trajectory = squeeze(pose_trajectories_kettle(1).pose_trajectory(1:3,4,:));
        plot3(position_trajectory(1,1:k),position_trajectory(2,1:k),position_trajectory(3,1:k),'r','LineWidth',2)
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
        
        subplot(2,2,2)
        % plot trajectory
        pose_trajectory = pose_trajectories_kettle(1).pose_trajectory;
        new_hom_vertices = pose_trajectory(:,:,k)*kettle.homogeneous_vertices;
        patch('vertices',new_hom_vertices(1:3,:)','faces',kettle.faces,'facecolor',kettle.color,'edgecolor','none','FaceAlpha',1)
        hold on
        p_ref = pose_trajectories_kettle(2).p_ref;
        scatter3(p_ref(1,1:k),p_ref(2,1:k),p_ref(3,1:k),5,'k','filled')
        hold on
        scatter3(p_ref(1,k),p_ref(2,k),p_ref(3,k),100,'k','filled')
        hold on
        position_trajectory = squeeze(pose_trajectories_kettle(2).pose_trajectory(1:3,4,:));
        plot3(position_trajectory(1,1:k),position_trajectory(2,1:k),position_trajectory(3,1:k),'g','LineWidth',2)
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
        p_ref = pose_trajectories_kettle(3).p_ref;
        scatter3(p_ref(1,1:k),p_ref(2,1:k),p_ref(3,1:k),5,'k','filled')
        hold on
        scatter3(p_ref(1,k),p_ref(2,k),p_ref(3,k),100,'k','filled')
        hold on
        position_trajectory = squeeze(pose_trajectories_kettle(3).pose_trajectory(1:3,4,:));
        plot3(position_trajectory(1,1:k),position_trajectory(2,1:k),position_trajectory(3,1:k),'b','LineWidth',2)
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
        new_hom_vertices = pose_trajectory(:,:,k)*bottle.homogeneous_vertices;
        patch('vertices',new_hom_vertices(1:3,:)','faces',bottle.faces,'facecolor',bottle.color,'edgecolor','none','FaceAlpha',1)
        hold on
        p_ref = pose_trajectories_bottle.p_ref;
        scatter3(p_ref(1,1:k),p_ref(2,1:k),p_ref(3,1:k),5,'k','filled')
        hold on
        scatter3(p_ref(1,k),p_ref(2,k),p_ref(3,k),100,'k','filled')
        hold on
        position_trajectory = squeeze(pose_trajectories_bottle.pose_trajectory(1:3,4,:));
        plot3(position_trajectory(1,1:k),position_trajectory(2,1:k),position_trajectory(3,1:k),'r','LineWidth',2)
        hold on
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
        set(gca,'FontSize',20)
        xlim([-0.8 0.3])
        ylim([-0.4 0.3])
        zlim([-0.2 0.4])
        
        drawnow
        
        pause(0.1)

    end
end