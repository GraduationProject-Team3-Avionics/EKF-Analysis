function log = load_vtol_log(csv_file, cfg)
%LOAD_VTOL_LOG Common STRIX/VTOL FC CSV loader.
%
% Usage:
%   log = load_vtol_log("");
%   log = load_vtol_log("fc_damp_04.CSV", cfg);
%
% This function does not save any files.

    if nargin < 1
        csv_file = "";
    end
    if nargin < 2
        cfg = struct();
    end

    if ~isfield(cfg, "analysis_start_sec"), cfg.analysis_start_sec = 0.0; end
    if ~isfield(cfg, "analysis_end_sec"),   cfg.analysis_end_sec = inf; end
    if ~isfield(cfg, "use_ekf_ready"),      cfg.use_ekf_ready = false; end
    if ~isfield(cfg, "use_gnss_ref_ready"), cfg.use_gnss_ref_ready = false; end
    if ~isfield(cfg, "use_gnss_valid"),     cfg.use_gnss_valid = false; end
    if ~isfield(cfg, "title_prefix"),       cfg.title_prefix = "STRIX FC Log"; end

    csv_file = string(csv_file);

    if strlength(csv_file) == 0
        csv_file = string(getenv("STRIX_FC_CSV"));
    end
    if strlength(csv_file) == 0
        csv_file = string(getenv("STRIX_PROP_CSV"));
    end
    if strlength(csv_file) == 0
        csv_file = string(getenv("STRIX_STAGE0_CSV"));
    end

    if strlength(csv_file) == 0
        [file_name, file_path] = uigetfile({"*.CSV;*.csv", "CSV files"}, ...
            "Select STRIX / VTOL FC log CSV");
        if isequal(file_name, 0)
            error("No CSV selected.");
        end
        csv_file = string(fullfile(file_path, file_name));
    end

    if ~isfile(csv_file)
        error("CSV file not found: %s", csv_file);
    end

    T = readtable(csv_file, "VariableNamingRule", "preserve");
    cols = string(T.Properties.VariableNames);
    N = height(T);

    strix_need(cols, "timestamp_ms");

    t = double(T.timestamp_ms) * 1.0e-3;
    t = t - t(1);

    dt = diff(t);
    dt_med = strix_medianf(dt);
    if isfinite(dt_med) && dt_med > 0
        fs_med = 1.0 / dt_med;
    else
        fs_med = NaN;
    end

    mask = isfinite(t) & t >= cfg.analysis_start_sec & t <= cfg.analysis_end_sec;

    if cfg.use_ekf_ready && ismember("ekf_ready", cols)
        mask = mask & strix_boolcol(T, cols, "ekf_ready", false(N, 1));
    end

    if cfg.use_gnss_ref_ready && ismember("gnss_ref_ready", cols)
        mask = mask & strix_boolcol(T, cols, "gnss_ref_ready", false(N, 1));
    end

    if cfg.use_gnss_valid && ismember("gnss_valid", cols)
        mask = mask & strix_boolcol(T, cols, "gnss_valid", false(N, 1));
    end

    log = struct();
    log.csv_file = csv_file;
    log.T = T;
    log.cols = cols;
    log.N = N;
    log.t = t;
    log.dt = dt;
    log.dt_median = dt_med;
    log.fs_median = fs_med;
    log.mask = mask;
    log.cfg = cfg;

    fprintf("\n=================================================\n");
    fprintf("%s\n", cfg.title_prefix);
    fprintf("=================================================\n");
    fprintf("CSV                : %s\n", csv_file);
    fprintf("Rows               : %d\n", N);
    fprintf("Columns            : %d\n", numel(cols));
    fprintf("Log time range     : %.3f ~ %.3f sec\n", strix_minf(t), strix_maxf(t));
    fprintf("Median dt / fs     : %.5f sec / %.2f Hz\n", dt_med, fs_med);
    fprintf("Mask samples       : %d / %d\n", sum(mask), N);
    fprintf("Mask time range    : %.3f ~ %.3f sec\n", strix_minf(t(mask)), strix_maxf(t(mask)));
    fprintf("=================================================\n\n");
end
