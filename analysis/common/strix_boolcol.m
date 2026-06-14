function y = strix_boolcol(T, cols, name, default_value)
%STRIX_BOOLCOL Read logical-ish table column or default.
    if ismember(name, cols)
        y = double(T.(name)) ~= 0;
    else
        y = default_value;
    end
end
