clear all

load('data_2021_04_07.mat')
data = load('data_2021_04_09.mat');


folder_names{end + 1} = data.folder_names{2};
folder_names{1} = [];
folder_names = folder_names(~cellfun('isempty',folder_names));
names{end + 1} = data.names{2};
names{1} = [];
names = names(~cellfun('isempty',names));

t{end + 1} = data.t{2};
t{1} = [];
t = t(~cellfun('isempty',t));
dist{end + 1} = data.dist{2};
dist{1} = [];
dist = dist(~cellfun('isempty',dist));
t_dist{end + 1} = data.t_dist{2};
t_dist{1} = [];
t_dist = t_dist(~cellfun('isempty',t_dist));
forces_unf{end + 1} = data.forces_unf{2};
forces_unf{1} = [];
forces_unf = forces_unf(~cellfun('isempty',forces_unf));
torques_unf{end + 1} = data.torques_unf{2};
torques_unf{1} = [];
torques_unf = torques_unf(~cellfun('isempty',torques_unf));
thetas{end + 1} = data.thetas{2};
thetas{1} = [];
thetas = thetas(~cellfun('isempty',thetas));
mocap_marker_robot_base{end + 1} = data.mocap_marker_robot_base{2};
mocap_marker_robot_base{1} = [];
mocap_marker_robot_base = mocap_marker_robot_base(~cellfun('isempty',mocap_marker_robot_base));

NO_BALL_BOUNC{end + 1} = data.NO_BALL_BOUNC{2};
NO_BALL_BOUNC{1} = [];
NO_BALL_BOUNC = NO_BALL_BOUNC(~cellfun('isempty',NO_BALL_BOUNC));
NO_DISTURBANCE{end + 1} = data.NO_DISTURBANCE{2};
NO_DISTURBANCE{1} = [];
NO_DISTURBANCE = NO_DISTURBANCE(~cellfun('isempty',NO_DISTURBANCE));
NO_FORCE_SENSOR{end + 1} = data.NO_FORCE_SENSOR{2};
NO_FORCE_SENSOR{1} = [];
NO_FORCE_SENSOR = NO_FORCE_SENSOR(~cellfun('isempty',NO_FORCE_SENSOR)); 
NO_GHOST_IMPULSE{end + 1} = data.NO_GHOST_IMPULSE{2};
NO_GHOST_IMPULSE{1} = [];
NO_GHOST_IMPULSE = NO_GHOST_IMPULSE(~cellfun('isempty',NO_GHOST_IMPULSE));
NO_IMPULSE{end + 1} = data.NO_IMPULSE{2};
NO_IMPULSE{1} = [];
NO_IMPULSE = NO_IMPULSE(~cellfun('isempty',NO_IMPULSE));
NO_MOCAP{end + 1} = data.NO_MOCAP{2};
NO_MOCAP{1} = [];
NO_MOCAP = NO_MOCAP(~cellfun('isempty',NO_MOCAP));
NO_TRQ_CMD_DIST{end + 1} = data.NO_TRQ_CMD_DIST{2};
NO_TRQ_CMD_DIST{1} = [];
NO_TRQ_CMD_DIST = NO_TRQ_CMD_DIST(~cellfun('isempty',NO_TRQ_CMD_DIST));
NO_VEL_CMD{end + 1} = data.NO_VEL_CMD{2};
NO_VEL_CMD{1} = [];
NO_VEL_CMD = NO_VEL_CMD(~cellfun('isempty',NO_VEL_CMD));

z_b{end + 1} = data.z_b{2};
z_b{1} = [];
z_b = z_b(~cellfun('isempty',z_b));
z_p{end + 1} = data.z_p{2};
z_p{1} = [];
z_p = z_p(~cellfun('isempty',z_p));
bounc_err{end + 1} = data.bounc_err{2};
bounc_err{1} = [];
bounc_err = bounc_err(~cellfun('isempty',bounc_err));

save("exp_1.mat", "dist", "dt", "folder_names", "forces_unf", ...
            "mocap_marker_robot_base", "NO_GHOST_IMPULSE",...
            "names", "NO_BALL_BOUNC", "NO_DISTURBANCE", "NO_IMPULSE", ...
            "NO_MOCAP", "NO_TRQ_CMD_DIST", "t", "t_dist", "thetas", ...
            "torques_unf", "z_b", "z_p", "bounc_err");