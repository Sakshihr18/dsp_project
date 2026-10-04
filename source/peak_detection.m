function peak_results = peak_detection(f, spectrum, char_freqs, peak_cfg)
% PEAK_DETECTION Detect significant spectral peaks and associate them with bearing characteristic frequencies and harmonics.
%
% DSP THEORY & ROLLER BEARING DYNAMICS:
%   1. Micro-slip in Rolling Bearings:
%      Kinematic formulas assume pure rolling without slipping. In actual
%      industrial bearings under dynamic loads, rolling elements experience
%      1% to 2% kinematic micro-slip. Therefore, observed fault peaks do not
%      align precisely at theoretical values, but rather within a narrow
%      frequency tolerance window: [f_char - Delta_f, f_char + Delta_f].
%
%   2. Significance Criteria (Avoiding False Alarms):
%      Random noise and turbulence produce small local maxima across the
%      spectrum. To identify genuine physical peaks:
%        - Peak Prominence: Measures how much a peak stands out relative to its
%          immediate local noise floor (adjacent valleys).
%        - Minimum Peak Distance: Enforces separation between distinct peaks.
%        - Relative Prominence: Compares peak prominence against the median
%          background level across the spectrum.
%
%   3. Multi-Harmonic Confirmation:
%      A true mechanical defect produces a periodic train of Dirac-like impacts.
%      In the frequency domain, periodic impulses generate an entire HARMONIC
%      COMB (1X, 2X, 3X). A peak detected at the fundamental that is also
%      supported by peaks at 2X and 3X provides strong DSP confirmation.
%
% INPUTS:
%   f           - Frequency vector [Hz]
%   spectrum    - Single-sided magnitude spectrum (FFT or Envelope spectrum)
%   char_freqs  - Struct of bearing characteristic frequencies from fault_frequencies.m
%   peak_cfg    - (Optional) Struct of peak detection parameters:
%                   .min_prominence   - Minimum peak prominence (default: 3 * median(spectrum))
%                   .min_distance_hz  - Minimum peak separation in Hz (default: 2.0 Hz)
%                   .tolerance_hz     - Search tolerance around fault frequencies in Hz (default: 2.5 Hz)
%                   .num_harmonics    - Number of harmonics to inspect (default: 3)
%
% OUTPUTS:
%   peak_results - Struct containing:
%                    .all_peaks       - Struct array of all detected significant peaks
%                    .matched_BPFO    - Detected peaks matching BPFO harmonics
%                    .matched_BPFI    - Detected peaks matching BPFI harmonics
%                    .matched_BSF     - Detected peaks matching BSF / 2xBSF harmonics
%                    .matched_FTF     - Detected peaks matching FTF harmonics
%                    .matched_fr      - Detected peaks matching shaft speed fr
%                    .noise_floor     - Estimated spectral noise floor (median)
%                    .peak_cfg_used   - Configuration used
%
% Author: DSP Project Team
% Project: Bearing Fault Diagnosis Using Motor Vibration Signals

    f = double(f(:));
    spectrum = double(spectrum(:));

    % 1. Default Configuration & Noise Floor Estimation
    noise_floor = median(spectrum);
    if noise_floor < 1e-12, noise_floor = 1e-12; end

    if nargin < 4 || isempty(peak_cfg)
        peak_cfg = struct();
    end

    if ~isfield(peak_cfg, 'min_prominence') || isempty(peak_cfg.min_prominence)
        % Adaptive prominence: at least 2.5x the median spectral noise floor
        peak_cfg.min_prominence = 2.5 * noise_floor;
    end
    if ~isfield(peak_cfg, 'min_distance_hz') || isempty(peak_cfg.min_distance_hz)
        peak_cfg.min_distance_hz = 2.0; % 2 Hz minimum separation
    end
    if ~isfield(peak_cfg, 'tolerance_hz') || isempty(peak_cfg.tolerance_hz)
        % Accommodate 1-2% roller micro-slip
        peak_cfg.tolerance_hz = 2.5; % +/- 2.5 Hz window
    end
    if ~isfield(peak_cfg, 'num_harmonics') || isempty(peak_cfg.num_harmonics)
        peak_cfg.num_harmonics = 3;
    end

    df = mean(diff(f));
    min_dist_samples = max(2, round(peak_cfg.min_distance_hz / df));

    % 2. Detect Peaks Using Signal Processing Toolbox or Fallback
    try
        [pks, locs, ~, proms] = findpeaks(spectrum, ...
            'MinPeakProminence', peak_cfg.min_prominence, ...
            'MinPeakDistance', min_dist_samples);
        peak_freqs = f(locs);
    catch
        % Fallback implementation for findpeaks
        [pks, locs, proms] = local_findpeaks_fallback(spectrum, min_dist_samples, peak_cfg.min_prominence);
        peak_freqs = f(locs);
    end

    % Assemble all significant peaks list
    num_detected = length(pks);
    all_peaks = struct('frequency', cell(num_detected, 1), ...
                       'magnitude', cell(num_detected, 1), ...
                       'prominence', cell(num_detected, 1), ...
                       'prom_to_noise', cell(num_detected, 1));
    for i = 1:num_detected
        all_peaks(i).frequency     = peak_freqs(i);
        all_peaks(i).magnitude     = pks(i);
        all_peaks(i).prominence    = proms(i);
        all_peaks(i).prom_to_noise = proms(i) / noise_floor;
    end

    % 3. Match Peaks with Bearing Kinematic Frequencies and Harmonics
    tol = peak_cfg.tolerance_hz;
    K   = peak_cfg.num_harmonics;

    peak_results.matched_BPFO = match_harmonics(char_freqs.BPFO, K, peak_freqs, pks, proms, tol, noise_floor);
    peak_results.matched_BPFI = match_harmonics(char_freqs.BPFI, K, peak_freqs, pks, proms, tol, noise_floor);
    
    % For ball defects, inspect both BSF (spin) and 2*BSF (ball defect impact rate)
    matched_BSF_raw = match_harmonics(char_freqs.BSF, K, peak_freqs, pks, proms, tol, noise_floor);
    matched_2BSF    = match_harmonics(char_freqs.twoBSF, K, peak_freqs, pks, proms, tol, noise_floor);
    peak_results.matched_BSF  = merge_ball_matches(matched_BSF_raw, matched_2BSF);

    peak_results.matched_FTF  = match_harmonics(char_freqs.FTF, 2, peak_freqs, pks, proms, tol, noise_floor);
    peak_results.matched_fr   = match_harmonics(char_freqs.fr, K, peak_freqs, pks, proms, tol, noise_floor);

    peak_results.all_peaks     = all_peaks;
    peak_results.noise_floor   = noise_floor;
    peak_results.peak_cfg_used = peak_cfg;
end

% =========================================================================
% HELPER FUNCTIONS
% =========================================================================

function matches = match_harmonics(f_target, K, peak_freqs, pks, proms, tol, noise_floor)
% Scan for detected peaks within +/- tol Hz of k * f_target
    matches = struct('harmonic_order', cell(K, 1), ...
                     'target_freq', cell(K, 1), ...
                     'detected_freq', cell(K, 1), ...
                     'magnitude', cell(K, 1), ...
                     'prominence', cell(K, 1), ...
                     'prom_to_noise', cell(K, 1), ...
                     'freq_error', cell(K, 1), ...
                     'is_detected', cell(K, 1));

    for k = 1:K
        f_expected = k * f_target;
        matches(k).harmonic_order = k;
        matches(k).target_freq    = f_expected;

        % Search within tolerance window
        in_band = abs(peak_freqs - f_expected) <= tol;
        if any(in_band)
            matching_indices = find(in_band);
            % If multiple peaks fall within window, pick the one with highest prominence
            [max_prom, best_sub_idx] = max(proms(matching_indices));
            best_idx = matching_indices(best_sub_idx);

            matches(k).detected_freq = peak_freqs(best_idx);
            matches(k).magnitude     = pks(best_idx);
            matches(k).prominence    = max_prom;
            matches(k).prom_to_noise = max_prom / noise_floor;
            matches(k).freq_error    = peak_freqs(best_idx) - f_expected;
            matches(k).is_detected   = true;
        else
            matches(k).detected_freq = NaN;
            matches(k).magnitude     = 0;
            matches(k).prominence    = 0;
            matches(k).prom_to_noise = 0;
            matches(k).freq_error    = NaN;
            matches(k).is_detected   = false;
        end
    end
end

function combined = merge_ball_matches(m_bsf, m_2bsf)
% Combines BSF and 2*BSF detections into a unified ball fault detection record
    combined.BSF_harmonics  = m_bsf;
    combined.twoBSF_harmonics = m_2bsf;
    % Mark detected if fundamental or 2*BSF is detected
    combined.is_detected = any([m_bsf.is_detected]) || any([m_2bsf.is_detected]);
    % Combined max prominence
    max_p_bsf  = max([m_bsf.prominence, 0]);
    max_p_2bsf = max([m_2bsf.prominence, 0]);
    combined.max_prominence = max(max_p_bsf, max_p_2bsf);
end

function [pks, locs, proms] = local_findpeaks_fallback(x, min_dist, min_prom)
% Pure MATLAB fallback for findpeaks when Signal Processing Toolbox is absent
    N = length(x);
    locs_temp = [];
    for i = 2:N-1
        if x(i) > x(i-1) && x(i) > x(i+1)
            locs_temp(end+1) = i; %#ok<AGROW>
        end
    end

    if isempty(locs_temp)
        pks = []; locs = []; proms = [];
        return;
    end

    % Prominence calculation: peak height minus maximum of local valleys
    proms_temp = zeros(size(locs_temp));
    for idx = 1:length(locs_temp)
        c_loc = locs_temp(idx);
        c_val = x(c_loc);
        % Left valley
        v_left = min(x(max(1, c_loc-min_dist*2):c_loc));
        % Right valley
        v_right = min(x(c_loc:min(N, c_loc+min_dist*2)));
        proms_temp(idx) = c_val - max(v_left, v_right);
    end

    % Filter by prominence
    valid = proms_temp >= min_prom;
    locs_cand = locs_temp(valid);
    proms_cand = proms_temp(valid);
    pks_cand  = x(locs_cand);

    % Distance suppression
    [~, sort_order] = sort(pks_cand, 'descend');
    keep = true(size(locs_cand));
    for i = 1:length(sort_order)
        curr = sort_order(i);
        if ~keep(curr), continue; end
        suppress = abs(locs_cand - locs_cand(curr)) < min_dist;
        suppress(curr) = false;
        keep(suppress) = false;
    end

    locs = locs_cand(keep);
    pks  = pks_cand(keep);
    proms = proms_cand(keep);
    [locs, sort_idx] = sort(locs);
    pks = pks(sort_idx);
    proms = proms(sort_idx);
end
