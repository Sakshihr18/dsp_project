%% =========================================================================
% BEARING FAULT DIAGNOSIS USING MOTOR VIBRATION SIGNALS
% Complete DSP-Based & Rule-Based Fault Diagnosis Pipeline (Zero Machine Learning)
%
% Theoretical Architecture:
%   CWRU Dataset File (.mat)
%            ↓
%   Signal Loading & Sensor Channel Inspection (DE / FE / BA)
%            ↓
%   Sampling Rate Configuration (48 kHz / 12 kHz)
%            ↓
%   Pre-Processing (DC Removal + Zero-Phase Butterworth Bandpass Filter)
%            ↓
%   Amplitude Normalization (Z-Score / Physical Unit Preservation)
%            ↓
%   Fixed-Length Segmentation (2048 Samples, 50% Overlap)
%            ↓
%   Hanning Windowing (Spectral Leakage Suppression)
%            ↓
%   Parallel DSP Feature Extraction:
%      ├── Time-Domain Statistical Moments (RMS, Kurtosis, Crest Factor, ...)
%      ├── Single-Sided Scaled FFT Analysis
%      ├── Welch Power Spectral Density (PSD)
%      ├── Hilbert Demodulation (Analytic Signal & Envelope Spectrum)
%      └── Bearing Kinematic Characteristic Frequencies (FTF, BPFO, BPFI, BSF)
%            ↓
%   Harmonic Comb & Adaptive Peak Detection (Findpeaks with Micro-Slip Tolerance)
%            ↓
%   Multi-Factor Rule-Based DSP Evidence Engine
%            ↓
%   Final Diagnostic Verdict:
%      [ Healthy Bearing / Inner Race Fault / Outer Race Fault / Ball Fault ]
%
% Author: DSP Project Team
% Project: Bearing Fault Diagnosis Using Motor Vibration Signals
%% =========================================================================

clear; clc; close all;

% Ensure current directory and subdirectories are on MATLAB path
script_dir = fileparts(mfilename('fullpath'));
if ~isempty(script_dir)
    addpath(script_dir);
end

fprintf('=======================================================================\n');
fprintf('       BEARING FAULT DIAGNOSIS USING MOTOR VIBRATION SIGNALS           \n');
fprintf('               Digital Signal Processing Implementation                \n');
fprintf('=======================================================================\n\n');


%% =========================================================================
% 1. USER CONFIGURATION SECTION
% =========================================================================

% --- Dataset File Configuration ---
% Point to the target CWRU .mat file inside the dataset folder.
% Example files:
%   '97.mat'  - Healthy baseline (48k DE, 1797 RPM, 0 HP load)
%   '105.mat' - Inner Race fault (12k DE, 0.007", 1797 RPM, 0 HP)
%   '118.mat' - Ball fault (12k DE, 0.007", 1797 RPM, 0 HP)
%   '130.mat' - Outer Race fault (12k DE, 0.007", 1797 RPM, 0 HP)
dataset_filename = '97.mat';

% Determine script directory robustly (handles Run Section, Editor, and Command Window execution)
if isempty(script_dir)
    script_dir = pwd;
end

% Automatically find dataset folder across candidate relative and project paths
candidates = {
    fullfile(script_dir, '..', 'dataset'), ...
    fullfile(script_dir, 'dataset'), ...
    fullfile(pwd, '..', 'dataset'), ...
    fullfile(pwd, 'dataset'), ...
    fullfile(pwd, 'dsp_project-main', 'dataset')
};

dataset_folder = '';
for k = 1:length(candidates)
    if isfile(fullfile(candidates{k}, dataset_filename))
        dataset_folder = candidates{k};
        break;
    end
end

if isempty(dataset_folder)
    path_hit = which(dataset_filename);
    if ~isempty(path_hit)
        dataset_folder = fileparts(path_hit);
    else
        dataset_folder = fullfile(script_dir, '..', 'dataset');
    end
end

dataset_path = fullfile(dataset_folder, dataset_filename);

% --- Sampling Frequency Configuration ---
% For 48k CWRU recordings, set Fs = 48000; for 12k recordings, set Fs = 12000.
Fs = 48000;

% --- Motor Shaft Speed Configuration ---
% CWRU standard motor operating speeds:
%   0 HP load: 1797 RPM (approx 29.95 Hz)
%   1 HP load: 1772 RPM (approx 29.53 Hz)
%   2 HP load: 1750 RPM (approx 29.17 Hz)
%   3 HP load: 1730 RPM (approx 28.83 Hz)
shaft_rpm = 1797;

% --- Sensor Channel Preference ---
% 'DE' (Drive End Accelerometer, recommended)
% 'FE' (Fan End Accelerometer)
% 'AUTO' (Auto-detect primary vibration channel)
channel_preference = 'DE';

% --- Bearing Kinematic Geometry: SKF 6205-2RS JEM (Drive End) ---
% Deep-groove ball bearing geometry specifications:
bearing_geom.Nb    = 9;      % Number of rolling elements (balls)
bearing_geom.d     = 7.940;  % Ball diameter in mm (0.3126 inches)
bearing_geom.D     = 39.04;  % Pitch diameter in mm (1.537 inches)
bearing_geom.theta = 0.0;    % Contact angle in degrees (0 deg for deep-groove)

% --- Pre-Processing Filter Parameters (Resonance Band-Pass) ---
% Isolates high-frequency structural resonance while attenuating low-frequency
% unbalance/misalignment and out-of-band sensor noise.
filter_cfg.order       = 4;     % 4th-order Butterworth bandpass filter
filter_cfg.low_cutoff  = 2000;  % Lower cutoff in Hz (resonance band start)
filter_cfg.high_cutoff = min(8000, 0.42 * Fs); % Upper cutoff in Hz (< Nyquist)

% --- Normalization Method ---
% 'zscore' (Standardization: zero-mean, unit-variance)
% 'peak'   (Full-scale [-1, +1] scaling)
% 'rms'    (Unit power scaling)
norm_method = 'zscore';

% --- Segmentation Parameters ---
window_size   = 2048; % Fixed-length analysis window (samples)
overlap_ratio = 0.50; % 50% overlap between adjacent segments

% --- Peak Detection Parameters ---
peak_cfg.tolerance_hz    = 2.5; % Frequency matching tolerance window (+/- Hz)
peak_cfg.min_distance_hz = 2.0; % Minimum separation between spectral peaks (Hz)
peak_cfg.num_harmonics   = 3;   % Number of harmonic multiples to inspect (1X, 2X, 3X)
peak_cfg.min_prominence  = [];  % Empty [] uses adaptive noise floor threshold

% --- Rule-Based Diagnostic Configuration ---
diag_cfg.fault_threshold        = 30.0; % Minimum evidence score to declare a fault
diag_cfg.healthy_kurtosis_limit = 3.6;  % Maximum kurtosis for Gaussian baseline

% --- Plotting Configuration ---
plot_cfg.max_freq_fft          = min(10000, Fs / 2); % Zoom limit for FFT (Hz)
plot_cfg.max_freq_env          = 500;                % Zoom limit for envelope (Hz)
plot_cfg.max_time_display_sec  = 0.5;                % Waveform zoom window (s)

%% =========================================================================
% 2. SIGNAL ACQUISITION & SAFE DATASET VERIFICATION
% =========================================================================

fprintf('[Step 1/11] Checking dataset file...\n');

if ~isfile(dataset_path)
    fprintf('\n========================================================================\n');
    fprintf('NOTICE: Dataset file not found!\n');
    fprintf('Target file path: %s\n\n', dataset_path);
    fprintf('Please place the CWRU .mat file inside the dataset folder and update\n');
    fprintf('the dataset_filename variable in the configuration section of main.m.\n\n');
    fprintf('Example:\n');
    fprintf('  1. Place "97.mat" inside: %s\n', fullfile(script_dir, '..', 'dataset'));
    fprintf('  2. Verify dataset_filename = ''97.mat'' at line 58 of main.m.\n');
    fprintf('  3. Re-run main.m.\n');
    fprintf('Refer to dataset/README.txt for complete instructions.\n');
    fprintf('========================================================================\n\n');
    return;
end

% Load raw vibration signal safely
[raw_signal, Fs_effective, channel_info] = load_signal(dataset_path, Fs, channel_preference);

% If RPM was detected directly from file metadata, use it to ensure precision
if ~isnan(channel_info.detected_rpm) && channel_info.detected_rpm > 1000
    shaft_rpm = channel_info.detected_rpm;
end

%% =========================================================================
% 3. CHARACTERISTIC FAULT FREQUENCY CALCULATION
% =========================================================================

fprintf('[Step 2/11] Computing bearing kinematic characteristic frequencies...\n');
char_freqs = fault_frequencies(bearing_geom, shaft_rpm, peak_cfg.num_harmonics);

%% =========================================================================
% 4. PRE-PROCESSING (DC REMOVAL + ZERO-PHASE BUTTERWORTH BAND-PASS)
% =========================================================================

fprintf('[Step 3/11] Applying DC removal and zero-phase Butterworth band-pass filter...\n');
[filtered_signal, filter_info] = preprocessing(raw_signal, Fs_effective, filter_cfg);

%% =========================================================================
% 5. NORMALIZATION (PRESERVING PHYSICAL METADATA)
% =========================================================================

fprintf('[Step 4/11] Performing signal normalization (%s)...\n', norm_method);
[norm_signal, norm_params] = normalization(filtered_signal, norm_method);

%% =========================================================================
% 6. FIXED-LENGTH SEGMENTATION (2048 SAMPLES, 50% OVERLAP)
% =========================================================================

fprintf('[Step 5/11] Segmenting signal (Window: %d, Overlap: %.0f%%)...\n', ...
    window_size, overlap_ratio * 100);
[segments, time_indices, seg_info] = segmentation(filtered_signal, window_size, overlap_ratio);

% Extract primary representative segment for windowed spectral inspection
primary_segment = segments(:, 1);

%% =========================================================================
% 7. TIME-DOMAIN STATISTICAL FEATURE EXTRACTION
% =========================================================================

fprintf('[Step 6/11] Calculating time-domain statistical descriptors...\n');
time_features = time_domain_features(filtered_signal);

%% =========================================================================
% 8. FAST FOURIER TRANSFORM (FFT) WITH HANNING WINDOWING
% =========================================================================

fprintf('[Step 7/11] Computing single-sided scaled FFT (with Hanning window)...\n');
[f_fft, mag_fft, dominant_peaks] = fft_analysis(primary_segment, Fs_effective, true, plot_cfg.max_freq_fft);

%% =========================================================================
% 9. POWER SPECTRAL DENSITY (WELCH'S METHOD)
% =========================================================================

fprintf('[Step 8/11] Computing Power Spectral Density via Welch method...\n');
[pxx, f_psd] = psd_analysis(filtered_signal, Fs_effective, window_size, ...
    round(window_size * overlap_ratio), window_size);

%% =========================================================================
% 10. HILBERT ENVELOPE ANALYSIS & DEMODULATED SPECTRUM
% =========================================================================

fprintf('[Step 9/11] Performing Hilbert envelope extraction and demodulation...\n');
[env_sig, env_spec, f_env] = envelope_analysis(filtered_signal, Fs_effective, true);
t_env = (0:length(env_sig)-1)' / Fs_effective;

%% =========================================================================
% 11. ADAPTIVE PEAK DETECTION & HARMONIC COMB MATCHING
% =========================================================================

fprintf('[Step 10/11] Detecting significant spectral peaks and matching harmonics...\n');
peak_results = peak_detection(f_env, env_spec, char_freqs, peak_cfg);

%% =========================================================================
% 12. RULE-BASED DSP FAULT DIAGNOSIS (ZERO MACHINE LEARNING)
% =========================================================================

fprintf('[Step 11/11] Executing rule-based DSP evidence diagnostic engine...\n\n');
diagnosis_report = diagnose_fault(peak_results, time_features, env_spec, f_env, char_freqs, diag_cfg);

%% =========================================================================
% 13. FORMATTED COMMAND WINDOW RESULTS DISPLAY
% =========================================================================

fprintf('--------------------------------------\n');
fprintf('BEARING FAULT DIAGNOSIS\n');
fprintf('--------------------------------------\n\n');

fprintf('Sampling Frequency: %d Hz\n', Fs_effective);
fprintf('RPM: %.1f\n\n', shaft_rpm);

fprintf('FTF:  %6.2f Hz\n', char_freqs.FTF);
fprintf('BPFO: %6.2f Hz\n', char_freqs.BPFO);
fprintf('BPFI: %6.2f Hz\n', char_freqs.BPFI);
fprintf('BSF:  %6.2f Hz\n\n', char_freqs.BSF);

fprintf('Time-Domain Features:\n');
fprintf('RMS:          %8.4f g\n', time_features.RMS);
fprintf('Peak:         %8.4f g\n', time_features.Peak);
fprintf('Peak-to-Peak: %8.4f g\n', time_features.PeakToPeak);
fprintf('Kurtosis:     %8.4f\n',   time_features.Kurtosis);
fprintf('Crest Factor: %8.4f\n',   time_features.CrestFactor);
fprintf('Skewness:     %8.4f\n\n', time_features.Skewness);

fprintf('Detected Peaks:\n');
if isfield(peak_results, 'all_peaks') && ~isempty(peak_results.all_peaks)
    all_p = peak_results.all_peaks;
    num_to_show = min(6, length(all_p));
    for pi = 1:num_to_show
        fprintf('  Peak %d: %6.2f Hz | Mag: %.4e | Prominence/Noise: %.1fx\n', ...
            pi, all_p(pi).frequency, all_p(pi).magnitude, all_p(pi).prom_to_noise);
    end
else
    fprintf('  No significant peaks detected exceeding noise threshold.\n');
end
fprintf('\n');

fprintf('Diagnostic Evidence:\n');
fprintf('BPFO score: %.1f / 100\n', diagnosis_report.EvidenceScores.BPFO);
fprintf('BPFI score: %.1f / 100\n', diagnosis_report.EvidenceScores.BPFI);
fprintf('BSF score:  %.1f / 100\n', diagnosis_report.EvidenceScores.BSF);
fprintf('FTF score:  %.1f / 100\n\n', diagnosis_report.EvidenceScores.FTF);

fprintf('Final Diagnosis:\n');
fprintf('>> %s <<\n', upper(diagnosis_report.FinalDiagnosis));
fprintf('(Confidence: %.1f%% | Severity: %s)\n', ...
    diagnosis_report.ConfidencePercent, diagnosis_report.Severity);
fprintf('--------------------------------------\n\n');

%% =========================================================================
% 14. GENERATE VISUALIZATIONS (9 DEDICATED FIGURES)
% =========================================================================

fprintf('Generating publication-quality diagnostic figures (1 to 9)...\n');
fig_handles = plot_results(raw_signal, filtered_signal, primary_segment, ...
                           f_fft, mag_fft, f_psd, pxx, ...
                           t_env, env_sig, f_env, env_spec, ...
                           char_freqs, peak_results, diagnosis_report, ...
                           Fs_effective, plot_cfg);

fprintf('DSP pipeline execution complete. All 9 figures generated.\n');
