function fig_handles = plot_results(raw_signal, filtered_signal, seg_signal, ...
                                    f_fft, mag_fft, f_psd, pxx, ...
                                    t_env, env_sig, f_env, env_spec, ...
                                    char_freqs, peak_results, diagnosis_report, ...
                                    Fs, plot_cfg)
% PLOT_RESULTS Generate publication-quality MATLAB diagnostic figures for the complete DSP pipeline.
%
% Generates 9 distinct, labeled figures:
%   Figure 1: Raw Vibration Signal
%   Figure 2: Filtered Signal (DC Removed + Butterworth Band-Pass)
%   Figure 3: Segmented & Hanning Windowed Signal
%   Figure 4: Single-Sided FFT Spectrum
%   Figure 5: Power Spectral Density (Welch's Method)
%   Figure 6: Envelope Signal (Hilbert Transform Demodulation)
%   Figure 7: Envelope Spectrum (Demodulated Baseband)
%   Figure 8: Envelope Spectrum with Kinematic Fault Markers (BPFO, BPFI, BSF, FTF)
%   Figure 9: Final Diagnostic Result (DSP Evidence Scores & Physical Summary)
%
% INPUTS:
%   raw_signal       - 1D raw vibration signal
%   filtered_signal  - 1D bandpass filtered signal
%   seg_signal       - 1D windowed segment (e.g. 2048 samples)
%   f_fft, mag_fft   - FFT frequency and magnitude vectors
%   f_psd, pxx       - PSD frequency and power density vectors
%   t_env, env_sig   - Envelope time and amplitude vectors
%   f_env, env_spec  - Envelope spectrum frequency and magnitude vectors
%   char_freqs       - Kinematic characteristic frequencies struct
%   peak_results     - Detected peaks struct
%   diagnosis_report - Fault diagnosis report struct
%   Fs               - Sampling rate in Hz
%   plot_cfg         - (Optional) Struct: .max_freq_fft, .max_freq_env, etc.
%
% OUTPUTS:
%   fig_handles      - Array of figure handles [1 x 9]
%
% Author: DSP Project Team
% Project: Bearing Fault Diagnosis Using Motor Vibration Signals

    if nargin < 16 || isempty(plot_cfg)
        plot_cfg = struct();
    end

    if ~isfield(plot_cfg, 'max_freq_fft') || isempty(plot_cfg.max_freq_fft)
        plot_cfg.max_freq_fft = min(12000, Fs / 2); % Zoomed view of FFT
    end
    if ~isfield(plot_cfg, 'max_freq_env') || isempty(plot_cfg.max_freq_env)
        plot_cfg.max_freq_env = 500; % Fault frequencies are typically < 500 Hz
    end
    if ~isfield(plot_cfg, 'max_time_display_sec') || isempty(plot_cfg.max_time_display_sec)
        plot_cfg.max_time_display_sec = 0.5; % Display first 0.5s for time clarity
    end

    fig_handles = gobjects(9, 1);

    % Color Palette Definition
    col_raw   = [0.20, 0.40, 0.70]; % Steel Blue
    col_filt  = [0.10, 0.60, 0.50]; % Teal Green
    col_win   = [0.85, 0.35, 0.10]; % Amber
    col_fft   = [0.30, 0.20, 0.60]; % Deep Purple
    col_psd   = [0.80, 0.20, 0.20]; % Crimson
    col_env   = [0.80, 0.45, 0.00]; % Dark Orange
    col_spec  = [0.10, 0.45, 0.80]; % Royal Blue

    % Time vectors
    N_raw = length(raw_signal);
    t_raw = (0:N_raw-1)' / Fs;

    N_filt = length(filtered_signal);
    t_filt = (0:N_filt-1)' / Fs;

    display_samples_raw  = min(N_raw, round(plot_cfg.max_time_display_sec * Fs));
    display_samples_filt = min(N_filt, round(plot_cfg.max_time_display_sec * Fs));

    % ---------------------------------------------------------------------
    % FIGURE 1: Raw Vibration Signal
    % ---------------------------------------------------------------------
    fig_handles(1) = figure('Name', 'Fig 1: Raw Vibration Signal', 'Color', 'w', 'Position', [100, 100, 800, 420]);
    plot(t_raw(1:display_samples_raw), raw_signal(1:display_samples_raw), 'Color', col_raw, 'LineWidth', 1.0);
    title('Raw Vibration Signal (Motor Accelerometer)', 'FontSize', 12, 'FontWeight', 'bold');
    xlabel('Time (s)', 'FontSize', 11);
    ylabel('Acceleration Amplitude (g)', 'FontSize', 11);
    grid on; box on;
    xlim([0, t_raw(display_samples_raw)]);

    % ---------------------------------------------------------------------
    % FIGURE 2: Filtered Signal
    % ---------------------------------------------------------------------
    fig_handles(2) = figure('Name', 'Fig 2: Filtered Signal', 'Color', 'w', 'Position', [120, 120, 800, 420]);
    plot(t_filt(1:display_samples_filt), filtered_signal(1:display_samples_filt), 'Color', col_filt, 'LineWidth', 1.0);
    title('Pre-Processed Signal (DC Removed + Zero-Phase Butterworth Band-Pass)', 'FontSize', 12, 'FontWeight', 'bold');
    xlabel('Time (s)', 'FontSize', 11);
    ylabel('Filtered Amplitude (g)', 'FontSize', 11);
    grid on; box on;
    xlim([0, t_filt(display_samples_filt)]);

    % ---------------------------------------------------------------------
    % FIGURE 3: Segmented and Windowed Signal
    % ---------------------------------------------------------------------
    fig_handles(3) = figure('Name', 'Fig 3: Segmented and Windowed Signal', 'Color', 'w', 'Position', [140, 140, 800, 420]);
    N_seg = length(seg_signal);
    t_seg = (0:N_seg-1)' / Fs;
    w_hann = 0.5 * (1 - cos(2 * pi * (0:N_seg-1)' / (N_seg - 1)));
    seg_windowed = seg_signal .* w_hann;

    plot(t_seg * 1000, seg_signal, 'Color', [0.6, 0.6, 0.6], 'LineWidth', 0.9, 'DisplayName', 'Segment (2048 samples)');
    hold on;
    plot(t_seg * 1000, seg_windowed, 'Color', col_win, 'LineWidth', 1.3, 'DisplayName', 'Hanning-Windowed Segment');
    plot(t_seg * 1000, max(abs(seg_signal)) * w_hann, '--k', 'LineWidth', 0.8, 'DisplayName', 'Hanning Envelope w(n)');
    hold off;
    title('2048-Sample Segment with Hanning Window (Leakage Suppression)', 'FontSize', 12, 'FontWeight', 'bold');
    xlabel('Segment Time (ms)', 'FontSize', 11);
    ylabel('Amplitude (g)', 'FontSize', 11);
    legend('Location', 'northeast');
    grid on; box on;

    % ---------------------------------------------------------------------
    % FIGURE 4: Single-Sided FFT Spectrum
    % ---------------------------------------------------------------------
    fig_handles(4) = figure('Name', 'Fig 4: FFT Spectrum', 'Color', 'w', 'Position', [160, 160, 800, 420]);
    fft_idx = (f_fft <= plot_cfg.max_freq_fft);
    plot(f_fft(fft_idx), mag_fft(fft_idx), 'Color', col_fft, 'LineWidth', 1.1);
    title(sprintf('Single-Sided Scaled FFT Magnitude Spectrum (0 - %.0f Hz)', plot_cfg.max_freq_fft), ...
        'FontSize', 12, 'FontWeight', 'bold');
    xlabel('Frequency (Hz)', 'FontSize', 11);
    ylabel('Magnitude Amplitude (g)', 'FontSize', 11);
    grid on; box on;
    xlim([0, plot_cfg.max_freq_fft]);

    % ---------------------------------------------------------------------
    % FIGURE 5: Welch Power Spectral Density
    % ---------------------------------------------------------------------
    fig_handles(5) = figure('Name', 'Fig 5: Welch PSD', 'Color', 'w', 'Position', [180, 180, 800, 420]);
    psd_idx = (f_psd <= plot_cfg.max_freq_fft);
    % Display in dB/Hz for dynamic range visibility
    pxx_db = 10 * log10(max(pxx(psd_idx), 1e-18));
    plot(f_psd(psd_idx), pxx_db, 'Color', col_psd, 'LineWidth', 1.2);
    title('Power Spectral Density Estimate (Welch Modified Periodogram)', 'FontSize', 12, 'FontWeight', 'bold');
    xlabel('Frequency (Hz)', 'FontSize', 11);
    ylabel('Power Density (dB / Hz)', 'FontSize', 11);
    grid on; box on;
    xlim([0, plot_cfg.max_freq_fft]);

    % ---------------------------------------------------------------------
    % FIGURE 6: Envelope Signal (Hilbert Demodulation)
    % ---------------------------------------------------------------------
    fig_handles(6) = figure('Name', 'Fig 6: Envelope Signal', 'Color', 'w', 'Position', [200, 200, 800, 420]);
    N_disp_env = min(length(env_sig), round(plot_cfg.max_time_display_sec * Fs));
    t_disp_env = (0:N_disp_env-1)' / Fs;

    plot(t_disp_env, filtered_signal(1:N_disp_env), 'Color', [0.75, 0.75, 0.75], 'LineWidth', 0.8, 'DisplayName', 'Filtered Carrier');
    hold on;
    plot(t_disp_env, env_sig(1:N_disp_env), 'Color', col_env, 'LineWidth', 1.3, 'DisplayName', 'Demodulated Envelope |z(t)|');
    hold off;
    title('Hilbert Demodulated Envelope Signal A(t) = |x(t) + j*H\{x(t)\}|', 'FontSize', 12, 'FontWeight', 'bold');
    xlabel('Time (s)', 'FontSize', 11);
    ylabel('Amplitude (g)', 'FontSize', 11);
    legend('Location', 'northeast');
    grid on; box on;
    xlim([0, t_disp_env(end)]);

    % ---------------------------------------------------------------------
    % FIGURE 7: Demodulated Envelope Spectrum
    % ---------------------------------------------------------------------
    fig_handles(7) = figure('Name', 'Fig 7: Envelope Spectrum', 'Color', 'w', 'Position', [220, 220, 800, 420]);
    env_idx = (f_env <= plot_cfg.max_freq_env);
    plot(f_env(env_idx), env_spec(env_idx), 'Color', col_spec, 'LineWidth', 1.2);
    title(sprintf('Demodulated Envelope Spectrum (Baseband: 0 - %.0f Hz)', plot_cfg.max_freq_env), ...
        'FontSize', 12, 'FontWeight', 'bold');
    xlabel('Modulation Frequency (Hz)', 'FontSize', 11);
    ylabel('Envelope Magnitude (g)', 'FontSize', 11);
    grid on; box on;
    xlim([0, plot_cfg.max_freq_env]);

    % ---------------------------------------------------------------------
    % FIGURE 8: Envelope Spectrum with Fault Markers (BPFO, BPFI, BSF, FTF)
    % ---------------------------------------------------------------------
    fig_handles(8) = figure('Name', 'Fig 8: Envelope Spectrum with Fault Markers', 'Color', 'w', 'Position', [240, 240, 860, 460]);
    plot(f_env(env_idx), env_spec(env_idx), 'Color', [0.2, 0.2, 0.2], 'LineWidth', 1.1, 'DisplayName', 'Envelope Spectrum');
    hold on;

    y_max = max(env_spec(env_idx)) * 1.15;
    if isempty(y_max) || y_max <= 0, y_max = 1; end

    % Characteristic frequency markers
    c_bpfo = [0.85, 0.00, 0.85]; % Magenta
    c_bpfi = [0.90, 0.15, 0.15]; % Red
    c_bsf  = [0.10, 0.70, 0.20]; % Green
    c_ftf  = [0.00, 0.70, 0.85]; % Cyan
    c_fr   = [0.20, 0.50, 0.90]; % Blue

    % Add vertical lines for fundamental and harmonics
    plot_freq_markers(char_freqs.BPFO, 3, c_bpfo, '--', 'BPFO', y_max, plot_cfg.max_freq_env);
    plot_freq_markers(char_freqs.BPFI, 3, c_bpfi, '-.', 'BPFI', y_max, plot_cfg.max_freq_env);
    plot_freq_markers(char_freqs.twoBSF, 2, c_bsf, ':', '2*BSF', y_max, plot_cfg.max_freq_env);
    plot_freq_markers(char_freqs.FTF, 2, c_ftf, ':', 'FTF', y_max, plot_cfg.max_freq_env);

    % Overlay detected significant peaks
    if isfield(peak_results, 'all_peaks') && ~isempty(peak_results.all_peaks)
        all_p = peak_results.all_peaks;
        p_freqs = [all_p.frequency];
        p_mags  = [all_p.magnitude];
        p_valid = (p_freqs <= plot_cfg.max_freq_env);
        if any(p_valid)
            scatter(p_freqs(p_valid), p_mags(p_valid), 45, 'r', 'filled', ...
                'DisplayName', 'Detected Significant Peaks');
        end
    end

    hold off;
    title('Envelope Spectrum with Bearing Characteristic Fault Frequencies', 'FontSize', 12, 'FontWeight', 'bold');
    xlabel('Frequency (Hz)', 'FontSize', 11);
    ylabel('Envelope Magnitude (g)', 'FontSize', 11);
    ylim([0, y_max]);
    xlim([0, plot_cfg.max_freq_env]);
    grid on; box on;
    legend('Location', 'northeast');

    % ---------------------------------------------------------------------
    % FIGURE 9: Final Diagnostic Result & Summary Dashboard
    % ---------------------------------------------------------------------
    fig_handles(9) = figure('Name', 'Fig 9: Diagnostic Result Dashboard', 'Color', 'w', 'Position', [260, 260, 920, 480]);

    % Subplot 1: Evidence Scores Bar Chart
    subplot(1, 2, 1);
    categories = {'BPFO\n(Outer Race)', 'BPFI\n(Inner Race)', 'BSF\n(Ball)', 'FTF\n(Cage)', 'Healthy\nBearing'};
    scores = [diagnosis_report.EvidenceScores.BPFO, ...
              diagnosis_report.EvidenceScores.BPFI, ...
              diagnosis_report.EvidenceScores.BSF, ...
              diagnosis_report.EvidenceScores.FTF, ...
              diagnosis_report.EvidenceScores.Healthy];

    b = bar(1:5, scores, 0.6);
    b.FaceColor = 'flat';
    b.CData(1, :) = c_bpfo;
    b.CData(2, :) = c_bpfi;
    b.CData(3, :) = c_bsf;
    b.CData(4, :) = c_ftf;
    b.CData(5, :) = [0.15, 0.65, 0.35]; % Green for healthy

    set(gca, 'XTick', 1:5, 'XTickLabel', {'BPFO', 'BPFI', 'BSF', 'FTF', 'Healthy'}, 'FontSize', 10);
    ylabel('Evidence Score (0 - 100)', 'FontSize', 11);
    title('DSP Fault Evidence Scores', 'FontSize', 12, 'FontWeight', 'bold');
    ylim([0, 110]);
    grid on;

    % Annotate bar values
    for bi = 1:5
        text(bi, scores(bi) + 3, sprintf('%.1f', scores(bi)), ...
            'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 9);
    end

    % Subplot 2: Diagnostic Verdict Summary Card
    subplot(1, 2, 2);
    axis off;

    % Determine banner color based on diagnosis
    if contains(diagnosis_report.FinalDiagnosis, 'Healthy')
        banner_color = [0.15, 0.65, 0.35]; % Green
    else
        banner_color = [0.85, 0.20, 0.15]; % Red / Orange
    end

    % Render Decision Banner Box
    rectangle('Position', [0.05, 0.72, 0.90, 0.24], 'FaceColor', banner_color, ...
              'EdgeColor', 'none', 'Curvature', [0.15, 0.15]);
    text(0.50, 0.88, 'FINAL DIAGNOSIS', 'Color', 'w', 'FontSize', 11, ...
         'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    text(0.50, 0.78, upper(diagnosis_report.FinalDiagnosis), 'Color', 'w', ...
         'FontSize', 13, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

    % Text summary of physical features
    tf = diagnosis_report.TimeFeatures;
    cf = diagnosis_report.CharFreqs;

    y_pos = 0.62;
    text(0.05, y_pos, 'Operating Parameters:', 'FontSize', 10, 'FontWeight', 'bold');
    y_pos = y_pos - 0.05;
    text(0.08, y_pos, sprintf('Shaft Speed: %.1f RPM (fr = %.2f Hz) | Fs: %d Hz', cf.rpm, cf.fr, Fs), 'FontSize', 9);

    y_pos = y_pos - 0.07;
    text(0.05, y_pos, 'Time-Domain Indicators:', 'FontSize', 10, 'FontWeight', 'bold');
    y_pos = y_pos - 0.05;
    text(0.08, y_pos, sprintf('RMS: %.3f g  |  Kurtosis: %.2f  |  Crest Factor: %.2f', ...
        tf.RMS, tf.Kurtosis, tf.CrestFactor), 'FontSize', 9);

    y_pos = y_pos - 0.07;
    text(0.05, y_pos, 'Diagnostic Confidence & Severity:', 'FontSize', 10, 'FontWeight', 'bold');
    y_pos = y_pos - 0.05;
    text(0.08, y_pos, sprintf('Confidence: %.1f%%  |  Severity: %s', ...
        diagnosis_report.ConfidencePercent, diagnosis_report.Severity), 'FontSize', 9);

    y_pos = y_pos - 0.07;
    text(0.05, y_pos, 'Key DSP Physical Evidence:', 'FontSize', 10, 'FontWeight', 'bold');
    reasons = diagnosis_report.SupportingEvidence;
    num_reasons = min(3, length(reasons));
    for ri = 1:num_reasons
        y_pos = y_pos - 0.045;
        % Truncate long lines if necessary
        msg = reasons{ri};
        if length(msg) > 65, msg = [msg(1:62), '...']; end
        text(0.08, y_pos, sprintf('- %s', msg), 'FontSize', 8);
    end
end

% =========================================================================
% PLOTTING HELPER FUNCTIONS
% =========================================================================

function plot_freq_markers(f_base, K, col, line_style, label_text, y_max, max_freq)
    for k = 1:K
        fk = k * f_base;
        if fk <= max_freq
            line([fk, fk], [0, y_max * 0.95], 'Color', col, 'LineStyle', line_style, 'LineWidth', 1.0);
            if k == 1
                text(fk + 1, y_max * 0.88, label_text, 'Color', col, 'FontSize', 8, 'FontWeight', 'bold');
            else
                text(fk + 1, y_max * 0.88, sprintf('%d*%s', k, label_text), 'Color', col, 'FontSize', 7);
            end
        end
    end
end
