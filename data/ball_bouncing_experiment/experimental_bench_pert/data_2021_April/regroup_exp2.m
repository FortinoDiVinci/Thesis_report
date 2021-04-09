clear all

load('data_2021_04_08.mat')

to_del = [1, 9, 10, 11];

for i = to_del
    folder_names{i} = [];
    names{i} = [];

    t{i} = [];
    t_ghost_impulse{i} = [];
    val_ghost_impulse{i} = [];
    t_impulse{i} = [];
    forces_unf{i} = [];
    torques_unf{i} = [];
    thetas{i} = [];
    mocap_marker_robot_base{i} = [];

    NO_BALL_BOUNC{i} = [];
    NO_DISTURBANCE{i} = [];
    NO_FORCE_SENSOR{i} = [];
    NO_GHOST_IMPULSE{i} = [];
    NO_IMPULSE{i} = [];
    NO_MOCAP{i} = [];
    NO_TRQ_CMD_DIST{i} = [];
    NO_VEL_CMD{i} = [];

    z_b{i} = [];
    z_p{i} = [];
    bounc_err{i} = [];

end
% to account the failed ghost recording of the first experiment
t_ghost_impulse{1} = [NaN];
val_ghost_impulse{1} = [NaN];

folder_names = folder_names(~cellfun('isempty',folder_names));
names = names(~cellfun('isempty',names));
t = t(~cellfun('isempty',t));
t_ghost_impulse = t_ghost_impulse(~cellfun('isempty',t_ghost_impulse));
val_ghost_impulse = val_ghost_impulse(~cellfun('isempty',val_ghost_impulse));
t_impulse = t_impulse(~cellfun('isempty',t_impulse));
forces_unf = forces_unf(~cellfun('isempty',forces_unf));
torques_unf = torques_unf(~cellfun('isempty',torques_unf));
thetas = thetas(~cellfun('isempty',thetas));
mocap_marker_robot_base = mocap_marker_robot_base(~cellfun('isempty',mocap_marker_robot_base));
NO_BALL_BOUNC = NO_BALL_BOUNC(~cellfun('isempty',NO_BALL_BOUNC));
NO_DISTURBANCE = NO_DISTURBANCE(~cellfun('isempty',NO_DISTURBANCE));
NO_FORCE_SENSOR = NO_FORCE_SENSOR(~cellfun('isempty',NO_FORCE_SENSOR)); 
NO_GHOST_IMPULSE = NO_GHOST_IMPULSE(~cellfun('isempty',NO_GHOST_IMPULSE));
NO_IMPULSE = NO_IMPULSE(~cellfun('isempty',NO_IMPULSE));
NO_MOCAP = NO_MOCAP(~cellfun('isempty',NO_MOCAP));
NO_TRQ_CMD_DIST = NO_TRQ_CMD_DIST(~cellfun('isempty',NO_TRQ_CMD_DIST));
NO_VEL_CMD = NO_VEL_CMD(~cellfun('isempty',NO_VEL_CMD));
bounc_err = bounc_err(~cellfun('isempty',bounc_err));
z_b = z_b(~cellfun('isempty',z_b));
z_p = z_p(~cellfun('isempty',z_p));

save("exp_2.mat", "dt", "folder_names", "forces_unf", ...
            "mocap_marker_robot_base", "NO_GHOST_IMPULSE",...
            "names", "NO_BALL_BOUNC", "NO_DISTURBANCE", "NO_IMPULSE", ...
            "NO_MOCAP", "NO_TRQ_CMD_DIST", "t", "thetas", ...
            "torques_unf", "z_b", "z_p", "bounc_err", "t_ghost_impulse",...
            "val_ghost_impulse", "t_impulse");