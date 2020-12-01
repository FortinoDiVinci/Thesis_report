function [total_time,total_ball_bounc,bouncing_err] = ball_bouncing_stats(t,z_b,vz_b,target_height)
%

    total_time = 0;
    total_ball_bounc = 0;
    bouncing_err = [];
    
    for fld_idx = 1:length(t)
        
        total_time = total_time + t{fld_idx}(end) - t{fld_idx}(1);
        
        null_vel_idx = find(diff(sign(vz_b{fld_idx})) < 0); % this way we only get ball apexes
        apex = find(diff(null_vel_idx) > 100); % apexes that are not too close
        if isempty(apex)
            continue
        end
        idx_apex = null_vel_idx(apex(1:end-3)); % the last impacts occurs while the experiment is stopping 
        
        total_ball_bounc = total_ball_bounc + length(idx_apex);
        if ~exist('new_exp_idx', 'var') 
            new_exp_idx = 1;
        else
            new_exp_idx = [new_exp_idx, length(bouncing_err)];
        end
        bouncing_err = [bouncing_err; (z_b{fld_idx}(idx_apex) - target_height)];

    end
    
    disp("For a total time of " + num2str(fix(total_time/60)) + "min.");
    disp(num2str(fix(total_ball_bounc)) + " bounces, and therefore cycles, with errors as follow:");
    disp("Mean bouncing error: " + num2str(mean(bouncing_err)./target_height*100, '%.2f') + "%");
    disp("Standard dev error: " + num2str(std(bouncing_err)./target_height*100, '%.2f') + "%");
    
    figure
    plot(bouncing_err)
    hold on
    plot(new_exp_idx, bouncing_err(new_exp_idx), 'p')
    title('Bouncing errors accross the trials (indicated by pentagones)')

end

