function saveAllFig(FolderName, i, thenCloseAll)

    if isstring(FolderName)
        FolderName = convertStringsToChars(FolderName);
    elseif ~ischar(FolderName)
        error('Please provide char array or string as input of the foler name')
    end

    if ~exist(FolderName, 'dir')
       mkdir(FolderName)
    end
    
    FigList = findobj(allchild(0), 'flat', 'Type', 'figure');
    openFigures = findobj('Type', 'figure');
    for iFig = 1:length(FigList)
        FigHandle = FigList(iFig);
        FigName = get(FigHandle, 'Name');
        %FigTitle = get(gca, 'title');
        %strTitle = get(FigTitle, 'string') + "_" + num2str(i) + "_" + num2str(iFig);
        di = length(openFigures(iFig).Children); % if there are both title and legend size is 2
        % if only title size is 1
        try
            strTitle = openFigures(iFig).Children(di).Title.String + "_" + num2str(i);
        catch
            strTitle = "default_title_"+num2str(iFig) + "_" + num2str(i);
        end
        savefig(FigHandle, fullfile(FolderName, FigName, convertStringsToChars(strTitle+".fig")));
        clear strTitle
    end
    
    if thenCloseAll
        close all
    end
    
end

