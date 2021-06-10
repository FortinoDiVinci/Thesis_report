load('exp_june_2021_ter')

users_list = ["418"; "456"; "495"; "546"; "548"; "573"; "640"; "661"; "666"; "000"];

selected_exp = zeros(length(exp_parameters),1);
% select users from list
for user_ref = users_list'
    selected_exp = selected_exp | [string(vertcat(exp_parameters.user)) == user_ref];
end
% selects exp 1 only
selected_exp = selected_exp & [string(vertcat(exp_parameters.experience)) == "exp_1"];


t_date = t_date(selected_exp);
t = t(selected_exp);
t_dist = t_d(selected_exp);
dist_val = dist_val(selected_exp);
q = q(selected_exp);
mocap_robot_endpoint = mocap_robot_endpoint(selected_exp);
ft_sensor = ft_sensor(selected_exp);
z_b = z_b(selected_exp);
z_p = z_p(selected_exp);
idx_ball_off_ramp = idx_ball_off_ramp(selected_exp);
bounce_err = bounce_err(selected_exp);
exp_parameters = exp_parameters(selected_exp);

data_000 = load('exp_june_000_2021');

selected_exp = zeros(length(data_000.exp_parameters),1);
% select users from list
for user_ref = users_list'
    selected_exp = selected_exp | [string(vertcat(data_000.exp_parameters.user)) == user_ref];
end
% selects exp 1 only
selected_exp = selected_exp & [string(vertcat(data_000.exp_parameters.experience)) == "exp_1"];

t_date = [t_date, data_000.t_date(selected_exp)];
t = [t, data_000.t(selected_exp)];
t_dist = [t_dist, data_000.t_d(selected_exp)];
dist_val = [dist_val, data_000.dist_val(selected_exp)];
q = [q, data_000.q(selected_exp)];
mocap_robot_endpoint = [mocap_robot_endpoint, data_000.mocap_robot_endpoint(selected_exp)];
ft_sensor = [ft_sensor, data_000.ft_sensor(selected_exp)];
z_b = [z_b, data_000.z_b(selected_exp)];
z_p = [z_p, data_000.z_p(selected_exp)];
idx_ball_off_ramp = [idx_ball_off_ramp, data_000.idx_ball_off_ramp(selected_exp)];
bounce_err = [bounce_err, data_000.bounce_err(selected_exp)];
exp_parameters = [exp_parameters, data_000.exp_parameters(selected_exp)];

save('SB2021_new_data_v2.mat', 't_date', 't', 't_dist', 'dist_val', 'q', ...
    'mocap_robot_endpoint', 'ft_sensor', 'z_b', 'z_p', 'idx_ball_off_ramp', ...
    'bounce_err', 'exp_parameters', 'users_list');

