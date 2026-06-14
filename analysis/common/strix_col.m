function y = strix_col(T, cols, name, default_value)
%STRIX_COL Read numeric table column or default.
    if ismember(name, cols)
        y = double(T.(name));
    else
        y = default_value;
    end
end
