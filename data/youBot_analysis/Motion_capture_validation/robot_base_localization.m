clear all

folder_name = 'geometric_data/';
dt = 1e-2; % 10ms

robot_endpoint = readtable(strcat(folder_name, 'bagfile-_vrpn_client_node_robot_marker_pose.csv'));
set_square = readtable(strcat(folder_name, 'bagfile-_vrpn_client_node_set_square_pose.csv'));

t1 = robot_endpoint.x_time*1e-9;
t2 = set_square.x_time*1e-9;

t_start = max(t1(1), t2(1));
t_end = min(t1(end), t2(end));

t = (t_start:dt:t_end)'-t_start;

% rob_xyz = [interp1(t1, robot_endpoint.field_pose_position_x, t), ...
%            interp1(t1, robot_endpoint.field_pose_position_y, t), ...
%            interp1(t1, robot_endpoint.field_pose_position_z, t)];
%        
% ssq_xyz = [interp1(t2, set_square.field_pose_position_x, t), ...
%            interp1(t2, set_square.field_pose_position_y, t), ...
%            interp1(t2, set_square.field_pose_position_z, t)];

ssq_xyz = [set_square.field_pose_position_x, ...
           set_square.field_pose_position_y, ...
           set_square.field_pose_position_z];

figure
hold on
plot3(ssq_xyz(:,1),ssq_xyz(:,2),ssq_xyz(:,3), '*')
xlabel('x(t)')
ylabel('y(t)')
zlabel('z(t)')

%% outliers manual deletion (comment if new data is used)

idx_outliers = find(ssq_xyz(:,2) <= -0.04 | ssq_xyz(:,2) >= -0.03);
ssq_xyz(idx_outliers,:) = [];
hold on
plot3(ssq_xyz(:,1),ssq_xyz(:,2),ssq_xyz(:,3), '*')

% cmatrix = ones(size(ssq_xyz)).*[1 0 0];
% ptCloud = pointCloud(ssq_xyz,'Color',cmatrix);
% maxDistance = 0.06;
% cylinder = pcfitcylinder(ptCloud,maxDistance);
% 
% figure
% pcshow(ptCloud)
% xlabel('X(m)')
% ylabel('Y(m)')
% zlabel('Z(m)')
% title('Detect a Cylinder in a Point Cloud')
% hold on
% plot(cylinder)

%% Cylinder identification from list of 3D points
% Tran T-T, et al. (2014) Extraction of cylinders and estimation of 
% their parameters from point clouds.

%% Points normal vectors computation

K_nghb = 50; % K closest neighbor
k = knnsearch(ssq_xyz,ssq_xyz, 'K', K_nghb); % finds 50 closest neighbors for each 
% points in the data set, k contains the indexes

% Hoppe H et al. (1992) Surface reconstruction from unorganized points.
normals = NaN(size(ssq_xyz));
eig_vector1 = NaN(size(ssq_xyz));
eig_vector2 = NaN(size(ssq_xyz));
eig_vector3 = NaN(size(ssq_xyz));
for i_pts = 1:length(ssq_xyz)    
    %m_pts(i_pts,:) = sum(ssq_xyz(k(i_pts,:),:))./K_nghb; % local centroid
    m_pts = sum(ssq_xyz(k(i_pts,:),:))./K_nghb; % local centroid
    CV = zeros(3,3);
    for j = 1:K_nghb
        p_j = ssq_xyz(k(i_pts,j),:);
        dj = p_j - m_pts;
        mu = exp( -(norm(dj)^2) / (K_nghb^2) ); % weight   
        CV = CV + mu*(dj')*dj; % covariance matrix computation
    end
    lambda = eig(CV);
    [eig_vectors,~] = eig(CV);
    normals(i_pts,:) = eig_vectors(:,logical(min(lambda) == lambda))'; % corresp. eig. vect.
    eig_vector1(i_pts,:) = eig_vectors(:,1)';
    eig_vector2(i_pts,:) = eig_vectors(:,2)';
    eig_vector3(i_pts,:) = eig_vectors(:,3)';
end

mean_normals = mean(normals);
mean_pts = mean(ssq_xyz);

q = quiver3(ssq_xyz(:,1),ssq_xyz(:,2),ssq_xyz(:,3), ssq_xyz(:,1)+normals(:,1), ...
    ssq_xyz(:,2)+normals(:,2), ssq_xyz(:,3)+normals(:,3));
%qm = quiver3(mean_pts(1), mean_pts(2), mean_pts(3), mean_pts(1)+mean_normals(1),...
%    mean_pts(2)+mean_normals(2), mean_pts(3)+mean_normals(3), 'Color', 'black', 'LineWidth', 1);

% figure
% plot3(ssq_xyz(:,1),ssq_xyz(:,2),ssq_xyz(:,3), '*')
% hold on
% q1 = quiver3(ssq_xyz(:,1),ssq_xyz(:,2),ssq_xyz(:,3), ssq_xyz(:,1)+eig_vector1(:,1), ...
%     ssq_xyz(:,2)+eig_vector1(:,2), ssq_xyz(:,3)+eig_vector1(:,3));
% q2 = quiver3(ssq_xyz(:,1),ssq_xyz(:,2),ssq_xyz(:,3), ssq_xyz(:,1)+eig_vector2(:,1), ...
%     ssq_xyz(:,2)+eig_vector2(:,2), ssq_xyz(:,3)+eig_vector2(:,3));
% q3 = quiver3(ssq_xyz(:,1),ssq_xyz(:,2),ssq_xyz(:,3), ssq_xyz(:,1)+eig_vector3(:,1), ...
%     ssq_xyz(:,2)+eig_vector3(:,2), ssq_xyz(:,3)+eig_vector3(:,3));
% legend([q1(1), q2(1), q3(1)],'Eig1','Eig2','Eig3')

%% Cylinder orientation

% we search the most orthogonal vector to all normals
CV_norm = zeros(3,3); % covariance matrix of all normal vectors
for i = 1:length(normals)
    CV_norm = CV_norm + (normals(i)')*normals(i);
end
lambda = eig(CV_norm);
[eig_vectors,~] = eig(CV_norm);
idx_norm_vect = find(min(lambda) == lambda); % min eigen value
a_cyl = eig_vectors(:,idx_norm_vect)'; % corresp. eig. vect.
q_c = quiver3(mean_pts(1), mean_pts(2), mean_pts(3), mean_pts(1)+a_cyl(1),...
    mean_pts(2)+a_cyl(2), mean_pts(3)+a_cyl(3), 'Color', 'black', 'LineWidth', 1, 'AutoScaleFactor', 0.05);

legend([q(1) q_c(1)], 'min(Eig\_vect)', 'cylinder')