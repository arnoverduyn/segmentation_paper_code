function plot_segmentation_result(pose_trajectories_kettle,parameters,bools)

    figure()
    if strcmp(bools.data_type,'real')
        colors = [0,0,1; %blue
            0,1,0; % green
            1,0,1; % m
            1,0,0; % red
            1,1,0; % yellow
            1,1,1];
        colors = [colors;colors/2;colors/3;colors/4];
    else
        colors = [0,1,0; %green
            1,0,1; %m
            1,0,0; %red
            1,1,0; % yellow
            0,1,1]; 
        colors = [colors;colors/2;colors/3;colors/4;colors/5];
    end

    for data = 1:parameters.data_size
        associated_cluster_indices = pose_trajectories_kettle(data).associated_cluster_indices;
        N = length(associated_cluster_indices);
        for i = 1:N
            if not(isnan(associated_cluster_indices(i)))
                color = colors(associated_cluster_indices(i),:);
            else
                color = [0,0,0];
            end
    %         scatter(i*0.02,0+(data-1),100,'filled','MarkerFaceColor',color)
            plot([(i-0.5)*0.02 (i+0.5)*0.02],[0-(data-1),0-(data-1)],'LineWidth',20,'color',color)
            hold on
        end
    end

    if strcmp(bools.evaluation_domain,'time')
        xlabel(' time [s] ')
    else
        xlabel(' s [m] ')
    end
    xlim([0 N*parameters.ds])
    
    if strcmp(bools.data_type,'real')
        ylim([-5.5 0.5])
    else
        ylim([-2.5 0.5])
    end
    
    set(gca, 'YTick', []);
    set(gca,'fontsize',15)
    box off

    if strcmp(bools.data_type,'simulation') && strcmp(bools.evaluation_domain,'time')
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
    end

end