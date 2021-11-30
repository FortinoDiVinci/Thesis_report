function cycles_n = regroupCompleteExperiment(cycles)
    % regroupCompleteExperiment:
    % When users failed to complete the entire duration of a single trial, they
    % had to complete it with another trial of the missing duration plus 30sec
    % this function regroups those trials
    % The list of failed trial by users was manualy fed.
    % In the end, each user is meant to have 5 trials

    users_name_f = ["041","160","208","211","278","311","378","495","546","640",...
        "661","710","841"];

    exp_id_f = {[1,4],[2,5],[3,6],[4,6],[1,2,5],2,5,4,1,1,2,[1,5],[1,3,7]}; % failed exp
    exp_if_c = {[2,5],[3,6],[4,7],[5,7],[2,3,6],3,6,5,2,2,3,[2,6],[2,4,8]}; % to merge with
    users_list = cellfun(@(x) x(1).user, cycles);

    for user_nb = 1:length(users_list)
        user = users_list(user_nb);
        cycle_i = cycles{user_nb};
        idx_user = find(strcmp(user, users_name_f));
        %
        for i = 1:length(cycle_i)
            cycle_i(i).exp_it_cmb = cycle_i(i).exp_it;
        end
        %
        if ~isempty(idx_user)
            failed_exp = exp_id_f{idx_user};
            combine_with = exp_if_c{idx_user};
            for i = 1:length(failed_exp)
                idx_up = find([cycle_i.exp_it] == combine_with(i), 1, 'first');
                for j = idx_up:length(cycle_i)
                    cycle_i(j).exp_it_cmb = cycle_i(j).exp_it_cmb - 1;
                end
            end
        end
        cycles_n{user_nb} = cycle_i;
        % max([cycles_n{user_nb}.exp_it_cmb])
    end

end

