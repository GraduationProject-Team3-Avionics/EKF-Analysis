function s = strix_status(pass_ok, warn_ok)
    if pass_ok
        s = "PASS";
    elseif warn_ok
        s = "WARN";
    else
        s = "FAIL";
    end
end
