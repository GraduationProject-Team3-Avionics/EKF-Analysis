function [acc_h, source] = strix_acc_ned_h(T, cols, N)
    if ismember("acc_ned_h", cols)
        acc_h = double(T.acc_ned_h);
        source = "acc_ned_h";
    elseif all(ismember(["acc_ned_n", "acc_ned_e"], cols))
        acc_h = hypot(double(T.acc_ned_n), double(T.acc_ned_e));
        source = "hypot(acc_ned_n, acc_ned_e)";
    else
        acc_h = nan(N, 1);
        source = "not available";
    end
end
