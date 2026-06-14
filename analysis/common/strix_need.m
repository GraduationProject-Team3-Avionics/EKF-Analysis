function strix_need(cols, names)
%STRIX_NEED Check required columns.
    missing = string.empty;
    names = string(names);
    for i = 1:numel(names)
        if ~ismember(names(i), cols)
            missing(end + 1) = names(i); %#ok<AGROW>
        end
    end
    if ~isempty(missing)
        error("Missing required column(s): %s", join(missing, ", "));
    end
end
