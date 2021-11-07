test_splines_interp_window
-------------
test_splines_interp_window_3:
K=400, B=10, M=0.5 ?? (check file)
estimated force (sine opt)
min interp = 200
max interp = 500
step = 1
--------------
test_splines_interp_window_3:
K=280, B=12, M=0.6
estimated force (sine opt)
min interp = 200
max interp = 350
step = 1
---------------
test_splines_interp_window_4:
K=280, B=12, M=0.6
simulated force (perturbation)
min interp = 200
max interp = 350
step = 5
---------------
spline_window_interp_id_500ms_1:
K=280, B=12, M=0.6
simulated force (perturbation)
min interp = 200
max interp = 500
step = 3
---------------
spline_window_interp_id_500ms_2:
K=539, B=44, M=2.8
simulated force (perturbation)
min interp = 200
max interp = 500
step = 3
---------------
spline_window_interp_id_500ms_b_1:
K=280, B=12, M=0.6
simulated force (perturbation)
alternative perturbation direction
min interp = 200
max interp = 500
step = 3
non symbolic identification
---------------
spline_window_interp_id_500ms_b_2:
K=539, B=44, M=2.8
simulated force (perturbation)
alternative perturbation direction
min interp = 200
max interp = 500
step = 3
non symbolic identification
----------------
spline_window_interp_id_500ms_c_1:
K=280, B=12, M=0.6
simulated force (perturbation)
min interp = 200
max interp = 500
step = 3
non symbolic identification
should be the same as:
spline_window_interp_id_500ms_1
-> validated
---------------
spline_window_interp_id_500ms_c_2:
K=539, B=44, M=2.8
simulated force (perturbation)
real virtual position
min interp = 200
max interp = 500
step = 3
non symbolic identification
should be the same as:
spline_window_interp_id_500ms_2
-> validated
----------------
spline_window_interp_id_500ms_d_1:
K=280, B=12, M=0.6
simulated force (perturbation)
real virtual position
min interp = 200
max interp = 500
step = 3
non symbolic identification
---------------
spline_window_interp_id_500ms_d_2:
K=539, B=44, M=2.8
simulated force (perturbation)
real virtual position
min interp = 200
max interp = 500
step = 3
non symbolic identification
----------------
spline_window_interp_id_500ms_e_1:
K=280, B=12, M=0.6
simulated force (perturbation)
both directions
real virtual position
perturbation mag = 10*Kv
min interp = 200
max interp = 500
step = 3
non symbolic identification
---------------
spline_window_interp_id_500ms_e_2:
K=539, B=44, M=2.8
simulated force (perturbation)
real virtual position
perturbation mag = 10*Kv
min interp = 200
max interp = 500
step = 3
non symbolic identification
----------------