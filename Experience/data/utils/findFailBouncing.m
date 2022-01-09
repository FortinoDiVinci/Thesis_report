function idx_i_red_new = findFailBouncing(zb, idx_i_red)

    zb = zb(idx_i_red(1):idx_i_red(end)); % reduced indexes
    %median(abs(diff(zb)-median(diff(zb))))    
    [~,outliers] = rmoutliers(diff(zb),'ThresholdFactor', 10);    
    idx_out = find(outliers); % outliers indexes
    
    if isempty(idx_out)
        % no outliers
        idx_i_red_new = idx_i_red;
        return
    end
    
    idx_new = find(idx_i_red >= (idx_out(end) + idx_i_red(1) - 1), 1, 'first');    
    idx_i_red_new = idx_i_red(idx_new:end);
    
%     figure
%     plot(t,zb)
%     hold on
%     plot(t(idx_out),zb(idx_out),'o')

end

