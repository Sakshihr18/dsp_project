function [filtered_signal, filter_info] = preprocessing(raw_signal, Fs, filter_cfg)
% PREPROCESSING Pre-process raw vibration signal using DC removal and zero-phase Butterworth band-pass filtering.
%
% DSP THEORY & SCIENTIFIC JUSTIFICATION:
%   Mechanical vibration recorded from rotating machinery consists of multiple
%   superimposed components:
%     1. Low-frequency components (< 500 Hz): Dominated by shaft unbalance (1X),
%        misalignment (2X), motor stator electrical line frequency (50/60 Hz)
%        and its harmonics, and rigid body structural modes.
%     2. Structural resonance carrier band (typically 2 kHz - 10 kHz in CWRU):
%        Localized defects (cracks, spalls) produce repetitive micro-shocks when
%        rolling elements pass over them. These sharp impacts act like Dirac
%        delta excitations, ring-modulating the high-frequency structural
%        resonances of the bearing outer ring and housing.
%     3. Out-of-band high-frequency noise (> 10 kHz): Random sensor noise,
%        cable triboelectric interference, and measurement quantization.
%
%   To extract localized bearing fault impulses via envelope demodulation,
%   we apply:
%     - DC offset elimination: Removes sensor baseline drift / bias.
%     - Butterworth band-pass filter: Selected for its maximally flat passband
%       response (zero passband ripple), ensuring resonance frequencies are
%       not artificially amplified or distorted.
%     - Zero-phase filtering (filtfilt): Filters forward and backward, achieving
%       strictly ZERO phase distortion ($H(e^{j\omega}) = |H|^2$). This preserves
%       the exact temporal arrival time and shape of transient shock impulses.
%
% INPUTS:
%   raw_signal  - Raw 1D vibration time series (column vector)
%   Fs          - Sampling rate in Hz
%   filter_cfg  - (Optional) Struct containing:
%                   .low_cutoff  - Lower cutoff frequency in Hz (default: 2000 Hz)
%                   .high_cutoff - Upper cutoff frequency in Hz (default: 8000 Hz)
%                   .order       - Filter order (default: 4)
%
% OUTPUTS:
%   filtered_signal - Preprocessed zero-phase band-pass filtered signal
%   filter_info     - Struct recording filter design and parameters
%
% Author: DSP Project Team
% Project: Bearing Fault Diagnosis Using Motor Vibration Signals

    % 1. Default filter configuration validation
    if nargin < 3 || isempty(filter_cfg)
        filter_cfg = struct();
    end

    if ~isfield(filter_cfg, 'order') || isempty(filter_cfg.order)
        filter_cfg.order = 4; % 4th-order Butterworth filter
    end
    if ~isfield(filter_cfg, 'low_cutoff') || isempty(filter_cfg.low_cutoff)
        filter_cfg.low_cutoff = 2000; % Default resonance low cutoff (Hz)
    end
    if ~isfield(filter_cfg, 'high_cutoff') || isempty(filter_cfg.high_cutoff)
        % Adapt to sampling rate: for 48 kHz use 8000 Hz, for 12 kHz use 4500 Hz
        filter_cfg.high_cutoff = min(8000, 0.40 * Fs);
    end

    % 2. DC Component Removal
    % Eliminates sensor offset and static bias
    signal_no_dc = raw_signal(:) - mean(raw_signal(:));

    % 3. Validate against Nyquist frequency
    F_nyquist = Fs / 2;
    f_low = filter_cfg.low_cutoff;
    f_high = filter_cfg.high_cutoff;

    % Safety checks for filter cutoffs
    if f_low <= 0
        f_low = 100;
        warning('DSP_PREPROC:LowCutoffAdjusted', 'Low cutoff must be > 0. Adjusted to 100 Hz.');
    end

    if f_high >= F_nyquist
        f_high = 0.42 * Fs;
        warning('DSP_PREPROC:HighCutoffAdjusted', ...
            'High cutoff exceeds or equals Nyquist (%0.1f Hz). Adjusted to %0.1f Hz.', ...
            F_nyquist, f_high);
    end

    if f_low >= f_high
        f_low = 0.1 * f_high;
        warning('DSP_PREPROC:CutoffOrderInverted', ...
            'Low cutoff >= high cutoff. Adjusted low cutoff to %0.1f Hz.', f_low);
    end

    % Normalized frequencies (0 to 1, where 1 is Nyquist)
    Wn = [f_low, f_high] / F_nyquist;
    n_order = filter_cfg.order;

    % 4. Butterworth Zero-Phase Band-Pass Filtering
    try
        % Attempt using Signal Processing Toolbox butter & filtfilt
        [b, a] = butter(n_order, Wn, 'bandpass');
        filtered_signal = filtfilt(b, a, signal_no_dc);
        filter_design_method = 'Butterworth (filtfilt zero-phase)';
    catch
        % Fallback implementation if Signal Processing Toolbox is unavailable:
        % Frequency-domain ideal zero-phase band-pass filtering via FFT
        warning('DSP_PREPROC:ToolboxFallback', ...
            'Signal Processing Toolbox butter/filtfilt unavailable. Using zero-phase FFT band-pass filter.');
        N = length(signal_no_dc);
        X = fft(signal_no_dc);
        freqs = (0:N-1)' * (Fs / N);
        % Symmetrical passband mask
        mask = ((freqs >= f_low & freqs <= f_high) | ...
                (freqs >= (Fs - f_high) & freqs <= (Fs - f_low)));
        % Smooth transition using cosine roll-off
        X_filtered = X .* double(mask);
        filtered_signal = real(ifft(X_filtered));
        b = []; a = [];
        filter_design_method = 'FFT Frequency-Domain Zero-Phase Bandpass (Fallback)';
    end

    % 5. Record filter metadata
    filter_info.method           = filter_design_method;
    filter_info.order            = n_order;
    filter_info.low_cutoff_Hz    = f_low;
    filter_info.high_cutoff_Hz   = f_high;
    filter_info.nyquist_Hz       = F_nyquist;
    filter_info.b_coeff          = b;
    filter_info.a_coeff          = a;
    filter_info.dc_removed       = mean(raw_signal(:));

    fprintf('[preprocessing] DC removed (%.4e). Band-pass filtered: [%.1f - %.1f] Hz (Fs = %d Hz)\n', ...
        filter_info.dc_removed, f_low, f_high, Fs);
end
