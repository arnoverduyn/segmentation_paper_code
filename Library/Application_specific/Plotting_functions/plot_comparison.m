function plot_comparison()

    figure()
    
    %%
    colors = [0.5,0.5,0.5; % gray 
    0,0.5,1; % light blue 
    0,1,1;   % light light blue
    0,0,1];  % blue 
     

    progress_domain = 'time';
    progress_type = 'time';
    L = 0;
    result = load(strcat('Output/Output_paper_v2/result_kettle_',progress_domain,'_',progress_type, '_', num2str(round(L*100)), 'cm'));
    pose_trajectories_kettle = result.pose_trajectories_kettle;
    
    for data = 1:3
        associated_cluster_indices = [pose_trajectories_kettle(data).associated_cluster_indices];
        N = length(associated_cluster_indices);
        new_time = 0:0.1:(N)*0.1;
        old_time = 0:0.02:(N)*0.1;
        associated_cluster_indices_wrt_time = NaN(1,length(old_time));
        counter = 1;
        for k = 1:N
            while counter <= length(old_time) && old_time(counter) < new_time(k)
                associated_cluster_indices_wrt_time(counter) = associated_cluster_indices(k);
                counter = counter + 1;
            end
        end    
        pose_trajectories_kettle(data).associated_cluster_indices_wrt_time = associated_cluster_indices_wrt_time;
    end

    for data = 1:3
        associated_cluster_indices_wrt_time = pose_trajectories_kettle(data).associated_cluster_indices_wrt_time;
        N = length(associated_cluster_indices_wrt_time);
        for i = 1:N
            if not(isnan(associated_cluster_indices_wrt_time(i)))
                color = colors(associated_cluster_indices_wrt_time(i),:);
            else
                color = [0,0,0];
            end
%             plot([(i+3)*0.02 (i+4)*0.02],[0-(data-1),0-(data-1)],'LineWidth',20,'color',color)
            plot([(i+4.5)*0.02 (i+5.5)*0.02],[0-(data-1),0-(data-1)],'LineWidth',15,'color',color)
            hold on
        end
    end
    
    %%
    colors = [0,0,1; % blue
    1,1,0; % yellow
    1,0,0; % red
    1,0.5,0]; % orange
    
    progress_domain = 'geometric';
    progress_type = 'arclength';
    L = 0;
    result = load(strcat('Output/Output_paper_v2/result_kettle_',progress_domain,'_',progress_type, '_', num2str(round(L*100)), 'cm'));
    pose_trajectories_kettle = result.pose_trajectories_kettle;
    
    for data = 1:3
        associated_cluster_indices = [NaN,NaN,pose_trajectories_kettle(data).associated_cluster_indices,NaN];
        s_t = pose_trajectories_kettle(data).s_t;
        N_t = size(s_t,2);
        xi = pose_trajectories_kettle(data).xi;
        N_s = size(xi,2);
        associated_cluster_indices_wrt_time = NaN(1,N_t);
        counter = 1;
        for k = 1:N_s
            while s_t(counter) < xi(k)
                associated_cluster_indices_wrt_time(counter) = associated_cluster_indices(k);
                counter = counter + 1;
            end
        end    
        pose_trajectories_kettle(data).associated_cluster_indices_wrt_time = associated_cluster_indices_wrt_time;
    end

    for data = 1:3
        associated_cluster_indices_wrt_time = pose_trajectories_kettle(data).associated_cluster_indices_wrt_time;
        N = length(associated_cluster_indices_wrt_time);
        for i = 1:N
            if not(isnan(associated_cluster_indices_wrt_time(i)))
                color = colors(associated_cluster_indices_wrt_time(i),:);
            else
                color = [0,0,0];
            end
    %         scatter(i*0.02,0+(data-1),100,'filled','MarkerFaceColor',color)
            plot([(i-0.5)*0.02 (i+0.5)*0.02],[-3.5-(data-1),-3.5-(data-1)],'LineWidth',15,'color',color)
            hold on
        end
    end
    
    %%
    colors = [0,0,1; % blue
    1,1,0 % yellow
    1,0,0; % red
    1,0.5,0]; % orange
    
    progress_domain = 'geometric';
    progress_type = 'arclength';
    L = 0.3;
    result = load(strcat('Output/Output_paper_v2/result_kettle_',progress_domain,'_',progress_type, '_', num2str(round(L*100)), 'cm'));
    pose_trajectories_kettle = result.pose_trajectories_kettle;
    
    for data = 1:3
        associated_cluster_indices = [NaN,NaN,pose_trajectories_kettle(data).associated_cluster_indices,NaN];
        s_t = pose_trajectories_kettle(data).s_t;
        N_t = size(s_t,2);
        xi = pose_trajectories_kettle(data).xi;
        N_s = size(xi,2);
        associated_cluster_indices_wrt_time = NaN(1,N_t);
        counter = 1;
        for k = 1:N_s
            while s_t(counter) < xi(k)
                associated_cluster_indices_wrt_time(counter) = associated_cluster_indices(k);
                counter = counter + 1;
            end
        end    
        pose_trajectories_kettle(data).associated_cluster_indices_wrt_time = associated_cluster_indices_wrt_time;
    end

    for data = 1:3
        associated_cluster_indices_wrt_time = pose_trajectories_kettle(data).associated_cluster_indices_wrt_time;
        N = length(associated_cluster_indices_wrt_time);
        for i = 1:N
            if not(isnan(associated_cluster_indices_wrt_time(i)))
                color = colors(associated_cluster_indices_wrt_time(i),:);
            else
                color = [0,0,0];
            end
    %         scatter(i*0.02,0+(data-1),100,'filled','MarkerFaceColor',color)
            plot([(i-0.5)*0.02 (i+0.5)*0.02],[-7-(data-1),-7-(data-1)],'LineWidth',15,'color',color)
            hold on
        end
    end
    
     %%
    colors = [1,1,0 % yellow
    1,0,1; % m
    1,0,0; % red
    1,0.5,0]; % orange
    
    progress_domain = 'geometric';
    progress_type = 'angle';
    L = 0.3;
    result = load(strcat('Output/Output_paper_v2/result_kettle_',progress_domain,'_',progress_type, '_', num2str(round(L*100)), 'cm'));
    pose_trajectories_kettle = result.pose_trajectories_kettle;
    
    for data = 1:3
        associated_cluster_indices = [NaN,NaN,pose_trajectories_kettle(data).associated_cluster_indices,NaN];
        s_t = pose_trajectories_kettle(data).s_t;
        N_t = size(s_t,2);
        xi = pose_trajectories_kettle(data).xi;
        N_s = size(xi,2);
        associated_cluster_indices_wrt_time = NaN(1,N_t);
        counter = 1;
        for k = 1:N_s
            while s_t(counter) < xi(k)
                associated_cluster_indices_wrt_time(counter) = associated_cluster_indices(k);
                counter = counter + 1;
            end
        end    
        pose_trajectories_kettle(data).associated_cluster_indices_wrt_time = associated_cluster_indices_wrt_time;
    end

    for data = 1:3
        associated_cluster_indices_wrt_time = pose_trajectories_kettle(data).associated_cluster_indices_wrt_time;
        N = length(associated_cluster_indices_wrt_time);
        for i = 1:N
            if not(isnan(associated_cluster_indices_wrt_time(i)))
                color = colors(associated_cluster_indices_wrt_time(i),:);
            else
                color = [0,0,0];
            end
    %         scatter(i*0.02,0+(data-1),100,'filled','MarkerFaceColor',color)
            plot([(i-0.5)*0.02 (i+0.5)*0.02],[-10.5-(data-1),-10.5-(data-1)],'LineWidth',15,'color',color)
            hold on
        end
    end
    
    
    %%
    colors = [0,0,1; % blue
        1,1,0; % yellow
        1,0,1; % m 
        1,0,0; % red
        0,1,0; % green
        1,0.5,0; % orange
        1,1,1];
    colors = [colors;colors/2;colors/3;colors/4];

    progress_domain = 'geometric';
    progress_type = 'combined';
    L = 0.3;
    result = load(strcat('Output/Output_paper_v2/result_kettle_',progress_domain,'_',progress_type, '_', num2str(round(L*100)), 'cm'));
    pose_trajectories_kettle = result.pose_trajectories_kettle;
    
    for data = 1:3
        associated_cluster_indices = [NaN,NaN,pose_trajectories_kettle(data).associated_cluster_indices,NaN];
        s_t = pose_trajectories_kettle(data).s_t;
        N_t = size(s_t,2);
        xi = pose_trajectories_kettle(data).xi;
        N_s = size(xi,2);
        associated_cluster_indices_wrt_time = NaN(1,N_t);
        counter = 1;
        for k = 1:N_s
            while s_t(counter) < xi(k)
                associated_cluster_indices_wrt_time(counter) = associated_cluster_indices(k);
                counter = counter + 1;
            end
        end    
        pose_trajectories_kettle(data).associated_cluster_indices_wrt_time = associated_cluster_indices_wrt_time;
    end

    for data = 1:3
        associated_cluster_indices_wrt_time = pose_trajectories_kettle(data).associated_cluster_indices_wrt_time;
        N = length(associated_cluster_indices_wrt_time);
        for i = 1:N
            if not(isnan(associated_cluster_indices_wrt_time(i)))
                color = colors(associated_cluster_indices_wrt_time(i),:);
            else
                color = [0,0,0];
            end
            plot([(i-0.5)*0.02 (i+0.5)*0.02],[-14-(data-1),-14-(data-1)],'LineWidth',15,'color',color)
            hold on
        end
    end
    
            %%
    colors = [0,1,0;
        1,0,1;
        1,0,0;
        1,1,0;
        1,1,1];
    colors = [colors;colors/2;colors/3;colors/4];

    progress_domain = 'geometric';
    progress_type = 'screw_based_old';
    L = 0.3;
    result = load(strcat('Output/Output_paper_v2/result_kettle_',progress_domain,'_',progress_type, '_', num2str(round(L*100)), 'cm'));
    pose_trajectories_kettle = result.pose_trajectories_kettle;
    
    for data = 1:3
        associated_cluster_indices = [NaN,NaN,pose_trajectories_kettle(data).associated_cluster_indices,NaN];
        s_t = pose_trajectories_kettle(data).s_t;
        N_t = size(s_t,2);
        xi = pose_trajectories_kettle(data).xi;
        N_s = size(xi,2);
        associated_cluster_indices_wrt_time = NaN(1,N_t);
        counter = 1;
        for k = 1:N_s
            while s_t(counter) < xi(k)
                associated_cluster_indices_wrt_time(counter) = associated_cluster_indices(k);
                counter = counter + 1;
            end
        end    
        pose_trajectories_kettle(data).associated_cluster_indices_wrt_time = associated_cluster_indices_wrt_time;
    end

    for data = 1:3
        associated_cluster_indices_wrt_time = pose_trajectories_kettle(data).associated_cluster_indices_wrt_time;
        N = length(associated_cluster_indices_wrt_time);
        for i = 1:N
            if not(isnan(associated_cluster_indices_wrt_time(i)))
                color = colors(associated_cluster_indices_wrt_time(i),:);
            else
                color = [0,0,0];
            end
            plot([(i-0.5)*0.02 (i+0.5)*0.02],[-17.5-(data-1),-17.5-(data-1)],'LineWidth',15,'color',color)
            hold on
        end
    end
    
    
        %%
    colors = [0,0,1;
        0,1,0;
        1,0,1;
        1,0,0;
        1,1,0;
        1,1,1];
    colors = [colors;colors/2;colors/3;colors/4];

    progress_domain = 'geometric';
    progress_type = 'screw_based_new';
    L = 0.3;
    result = load(strcat('Output/Output_paper_v2/result_kettle_',progress_domain,'_',progress_type, '_', num2str(round(L*100)), 'cm'));
    pose_trajectories_kettle = result.pose_trajectories_kettle;
    
    for data = 1:3
        associated_cluster_indices = [NaN,NaN,pose_trajectories_kettle(data).associated_cluster_indices,NaN];
        s_t = pose_trajectories_kettle(data).s_t;
        N_t = size(s_t,2);
        xi = pose_trajectories_kettle(data).xi;
        N_s = size(xi,2);
        associated_cluster_indices_wrt_time = NaN(1,N_t);
        counter = 1;
        for k = 1:N_s
            while s_t(counter) < xi(k)
                associated_cluster_indices_wrt_time(counter) = associated_cluster_indices(k);
                counter = counter + 1;
            end
        end    
        pose_trajectories_kettle(data).associated_cluster_indices_wrt_time = associated_cluster_indices_wrt_time;
    end

    for data = 1:3
        associated_cluster_indices_wrt_time = pose_trajectories_kettle(data).associated_cluster_indices_wrt_time;
        N = length(associated_cluster_indices_wrt_time);
        for i = 1:N
            if not(isnan(associated_cluster_indices_wrt_time(i)))
                color = colors(associated_cluster_indices_wrt_time(i),:);
            else
                color = [0,0,0];
            end
            plot([(i-0.5)*0.02 (i+0.5)*0.02],[-21-(data-1),-21-(data-1)],'LineWidth',15,'color',color)
            hold on
        end
    end

    
    xline(2,'-k','LineWidth',2) % start segment 1
    hold on
    xline(5,'-k','LineWidth',2) % start segment 2
    hold on
    xline(7,'-k','LineWidth',2) % start segment 3
    hold on
    xline(11,'-k','LineWidth',2) % start segment 4
    hold on
    xline(13,'-k','LineWidth',2) % start segment 5
    hold on
    xline(15,'-k','LineWidth',2) % start segment 6
    hold on
    xline(17,'-k','LineWidth',2) % start standstill
    hold on
    
    xlim([0 17.5])
    xlabel(' time [s] ')
    ylim([-24 2])
    set(gca, 'YTick', []);
    set(gca,'fontsize',15)
    box off

end