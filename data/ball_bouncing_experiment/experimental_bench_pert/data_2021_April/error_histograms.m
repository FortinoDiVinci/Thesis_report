function [users_median, users_25prc, users_75prc] = error_histograms(errors,names)

% regroup data by user reference name
names_regr = names;
regrouped = [];
errors_n = errors;
for ii = 1:length(errors)
    if any(ii == regrouped)
        names_regr(ii) = "";
        continue
    end
    tmp = names;
    tmp(ii) = [""]; % mask the iith name to avoid seing itself as duplicate
    string_compare = strcmp(names(ii), tmp);
    idx = find(string_compare);
    if any(string_compare)
        for i_idx = idx'
            regrouped = [regrouped, i_idx];
            errors_n{ii} = horzcat(errors{ii}, errors{i_idx});
            errors_n{i_idx} = [];
        end
        
    end
end
% remove empty cells
errors_n =  errors_n(~cellfun('isempty',errors_n));
names_regr = names_regr(~(names_regr==""));

[sp,~] = numSubplots(length(errors_n));
users_prc70 = zeros(size(errors_n));
users_median = zeros(size(errors_n));

figure('DefaultAxesFontSize',16)
%tiledlayout('flow', 'TileSpacing', 'compact', 'Padding', 'compact')
tiledlayout(sp(1),sp(2), 'TileSpacing', 'compact', 'Padding', 'compact')
for ii = 1:length(errors_n)
    nexttile
    try
        errs = vertcat(errors_n{ii}.data);
    catch
        errs = horzcat(errors_n{ii}.data);
    end
    prc_data = prctile(errs,[25,50,75]);
    hold on
    histogram(errs, 'BinWidth', 0.05)
    xline(prc_data(2),'k--','LineWidth',2)
    xline(prc_data(1),'r--','LineWidth',2)
    xline(prc_data(3),'r--','LineWidth',2)
    title("user#" + names_regr(ii))
    users_25prc(ii) = prc_data(1);
    users_75prc(ii) = prc_data(3); 
    users_median(ii) = prc_data(2);
end



function [p,n]=numSubplots(n)
% function [p,n]=numSubplots(n)
%
% Purpose
% Calculate how many rows and columns of sub-plots are needed to
% neatly display n subplots. 
%
% Inputs
% n - the desired number of subplots.     
%  
% Outputs
% p - a vector length 2 defining the number of rows and number of
%     columns required to show n plots.     
% [ n - the current number of subplots. This output is used only by
%       this function for a recursive call.]
%
%
%
% Example: neatly lay out 13 sub-plots
% >> p=numSubplots(13)
% p = 
%     3   5
% for i=1:13; subplot(p(1),p(2),i), pcolor(rand(10)), end 
%
%
% Rob Campbell - January 2010
   
    
while isprime(n) & n>4, 
    n=n+1;
end
p=factor(n);
if length(p)==1
    p=[1,p];
    return
end
while length(p)>2
    if length(p)>=4
        p(1)=p(1)*p(end-1);
        p(2)=p(2)*p(end);
        p(end-1:end)=[];
    else
        p(1)=p(1)*p(2);
        p(2)=[];
    end    
    p=sort(p);
end
%Reformat if the column/row ratio is too large: we want a roughly
%square design 
while p(2)/p(1)>2.5
    N=n+1;
    [p,n]=numSubplots(N); %Recursive!
end    