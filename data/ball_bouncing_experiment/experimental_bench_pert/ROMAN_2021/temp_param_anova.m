di1 = data_sorted(1).pos.K(~idx_tot{1},1);
di2 = data_sorted(1).neg.K(~idx_tot{2},1);
di3 = data_sorted(2).pos.K(~idx_tot{3},1);
di4 = data_sorted(2).neg.K(~idx_tot{4},1);
di5 = data_sorted(3).pos.K(~idx_tot{5},1);
di6 = data_sorted(3).neg.K(~idx_tot{6},1);

di_all = [di1;di2;di3;di4;di5;di6];
di_all_gr = [ones(size(di1)); 2.*ones(size(di2)); 3.*ones(size(di3)); 4.*ones(size(di4)); 5.*ones(size(di5)); 6.*ones(size(di6))];
anova1(di_all, di_all_gr)

dipos = [di1;di3;di5];
dineg = [di2;di4;di6];
dipn_gr = [ones(size(dipos)); 2.*ones(size(dineg))];
anova1([dipos; dineg], dipn_gr)


dir1 = data_sorted(1).pos.r2(~idx_tot{1},1);
dir3 = data_sorted(2).pos.r2(~idx_tot{3},1);
dir5 = data_sorted(3).pos.r2(~idx_tot{5},1);
dir2 = data_sorted(1).neg.r2(~idx_tot{2},1);
dir4 = data_sorted(2).neg.r2(~idx_tot{4},1);
dir6 = data_sorted(3).neg.r2(~idx_tot{6},1);
dir_all = [dir1;dir2;dir3;dir4;dir5;dir6];
anova1(dir_all, di_all_gr)

dirpos = [dir1;dir3;dir5];
dirneg = [dir2;dir4;dir6];
anova1([dirpos; dirneg], dipn_gr)

dib1 = data_sorted(1).pos.B(~idx_tot{1},1);
dib3 = data_sorted(2).pos.B(~idx_tot{3},1);
dib5 = data_sorted(3).pos.B(~idx_tot{5},1);
dib2 = data_sorted(1).neg.B(~idx_tot{2},1);
dib4 = data_sorted(2).neg.B(~idx_tot{4},1);
dib6 = data_sorted(3).neg.B(~idx_tot{6},1);
dib_all = [dib1;dib2;dib3;dib4;dib5;dib6];
anova1([dib1;dib2;dib3;dib4;dib5;dib6], di_all_gr)

dim1 = data_sorted(1).pos.M(~idx_tot{1},1);
dim3 = data_sorted(2).pos.M(~idx_tot{3},1);
dim5 = data_sorted(3).pos.M(~idx_tot{5},1);
dim2 = data_sorted(1).neg.M(~idx_tot{2},1);
dim4 = data_sorted(2).neg.M(~idx_tot{4},1);
dim6 = data_sorted(3).neg.M(~idx_tot{6},1);
dim_all = [dim1;dim2;dim3;dim4;dim5;dim6];
anova1([dim1;dim2;dim3;dim4;dim5;dim6], di_all_gr)


data200ms = load('data_tmp_anova_200ms.mat');

% R2
anova1([dir_all; data200ms.dr_all], [ones(size(dir_all)); 2.*ones(size(data200ms.dr_all))])
% K
anova1([di_all; data200ms.d_all], [ones(size(di_all)); 2.*ones(size(data200ms.d_all))])
% B
data200ms.db_all = [data200ms.db1; data200ms.db2; data200ms.db3; data200ms.db4; data200ms.db5; data200ms.db6];
anova1([dib_all; data200ms.db_all], [ones(size(dib_all)); 2.*ones(size(data200ms.db_all))])
% M
data200ms.dm_all = [data200ms.dm1; data200ms.dm2; data200ms.dm3; data200ms.dm4; data200ms.dm5; data200ms.dm6];
anova1([dim_all; data200ms.dm_all], [ones(size(dim_all)); 2.*ones(size(data200ms.dm_all))])


% clear data
% 
% save('data_tmp_anova_200ms.mat')