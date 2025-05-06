function save_simulation_data(pose_trajectories_kettle,pose_trajectories_bottle,bools)
    if strcmp(bools.data_type,'new_simulation') && bools.save_simulation_data
        for data = 1:3
            simulation_data.pose_trajectories_kettle(data).pose_trajectory = pose_trajectories_kettle(data).pose_trajectory;
        end
        simulation_data.pose_trajectories_bottle.pose_trajectory = pose_trajectories_bottle.pose_trajectory;
        save('Output/simulation_data','simulation_data')
    end
end