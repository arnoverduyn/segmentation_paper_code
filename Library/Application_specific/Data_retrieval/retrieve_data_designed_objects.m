function [kettle,bottle,table,cup] = retrieve_data_designed_objects()

% kettle
rp = stlread("objects/kettle.stl");
vertices = rp.Points/1500; % scale down + convert to [m]
vertices(:,3) = vertices(:,3) - 0.02; % calibrate ref point to be near CoM
vertices = transpose(rotz(-125)*vertices');
kettle.faces = rp.ConnectivityList;
kettle.number_of_vertices = size(vertices,1);
kettle.homogeneous_vertices = [vertices';ones(1,kettle.number_of_vertices)];
kettle.color = [0 0.5 1];

% bottle
rp = stlread("objects/bottle.stl");
vertices = rp.Points/1500; % scale down + convert to [m]
vertices(:,3) = vertices(:,3) - 0.18; % calibrate ref point to be near spout
vertices = transpose(rotz(-90)*vertices');
bottle.faces = rp.ConnectivityList;
bottle.number_of_vertices = size(vertices,1);
bottle.homogeneous_vertices = [vertices';ones(1,bottle.number_of_vertices)];
bottle.color = [1 0.2 0.2];

% table
rp = stlread("objects/table.stl");
vertices = rp.Points/1000; % convert to [m]
vertices(:,1) = vertices(:,1) - 0.3; 
vertices(:,2) = vertices(:,2) - 0.05; 
vertices(:,3) = vertices(:,3) - 0.02;
table.faces = rp.ConnectivityList;
table.number_of_vertices = size(vertices,1);
table.homogeneous_vertices = [vertices';ones(1,table.number_of_vertices)];
table.color = [80 40 40]./150;

% cup
rp = stlread("objects/cup.stl");
vertices = rp.Points/2000; % convert to [m]
vertices(:,1) = vertices(:,1) - 0.54;  % case kettle, for bottle pouring, translate x-position with + 0.05 m
vertices(:,2) = vertices(:,2) - 0.09; 
vertices(:,3) = vertices(:,3) - 0.02;
cup.faces = rp.ConnectivityList;
cup.number_of_vertices = size(vertices,1);
cup.homogeneous_vertices = [vertices';ones(1,cup.number_of_vertices)];
cup.color = [0 1 0];

end