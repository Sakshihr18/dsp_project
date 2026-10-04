function [pxx, f] = psd_analysis(signal, Fs, window_len, noverlap, nfft)
% PSD_ANALYSIS Compute Power Spectral Density (PSD) using Welch's modified periodogram averaging method.
%
% DSP THEORY & JUSTIFICATION:
%   A single standard periodogram computed from raw vibration is an inconsistent
%   spectral estimator: as signal length N increases, variance does not converge
%   to zero.
%
%   Welch's method solves this through three DSP principles:
%     1. Data partitioning: Divides the time series into overlapping segments.
%     2. Tapered windowing (Hann): Pre-multiplies each segment by a Hann window
%        to suppress spectral leakage from segment truncation.
%     3. Periodogram averaging: Averages the modified periodograms across all
%        segments. Averaging K independent segments reduces the variance of
%        the spectral estimate by approximately a factor of 1/K, yielding a
%        smooth, reliable power distribution curve:
%
%        P_welch(f) = (1 / K) * sum_{k=1}^K P_k(f)
%        where P_k(f) = (1 / (Fs * U)) * |FFT{ x_k(n) .* w(n) }|^2
%        and U = (1/L) * sum( w(n)^2 ) is the window power normalization factor.
%
%   Units of output:
%     Power / Hz (e.g. g^2 / Hz or V^2 / Hz), representing spectral energy density.
%
% INPUTS:
%   signal     - 1D vibration signal vector
%   Fs         - Sampling frequency in Hz
%   window_len - (Optional) Window length in samples (default: min(2048, length(signal)))
%   noverlap   - (Optional) Overlap samples (default: 50% of window_len)
%   nfft       - (Optional) FFT length (default: nextpow2(window_len) or window_len)
%
% OUTPUTS:
%   pxx - Power Spectral Density vector in Power/Hz (e.g., g^2/Hz)
%   f   - Frequency vector in Hz [0, Fs/2]
%
% Author: DSP Project Team
% Project: Bearing Fault Diagnosis Using Motor Vibration Signals

    x = double(signal(:));
    N = length(x);

    if N < 16
        error('DSP_PSD:SignalTooShort', 'Signal length must be >= 16 samples for PSD analysis.');
    end

    % Configure default Welch parameters
    if nargin < 3 || isempty(window_len)
        window_len = min(2048, 2^floor(log2(N)));
        if window_len < 64, window_len = N; end
    end
    if nargin < 4 || isempty(noverlap)
        noverlap = floor(window_len / 2); % 50% overlap
    end
    if nargin < 5 || isempty(nfft)
        nfft = max(256, 2^nextpow2(window_len));
    end

    % Hann window for each segment
    w = 0.5 * (1 - cos(2 * pi * (0:window_len-1)' / (window_len - 1)));

    % Try using Signal Processing Toolbox pwelch
    try
        [pxx, f] = pwelch(x, w, noverlap, nfft, Fs);
    catch
        % Fallback implementation: Welch's method computed from pure MATLAB
        % to guarantee operation on machines without Signal Processing Toolbox
        step = window_len - noverlap;
        num_segs = floor((N - window_len) / step) + 1;
        if num_segs < 1
            num_segs = 1;
            window_len = N;
            w = 0.5 * (1 - cos(2 * pi * (0:window_len-1)' / (window_len - 1)));
            step = 1;
        end

        U = sum(w.^2); % Window power normalization
        num_unique_pts = floor(nfft / 2) + 1;
        pxx_accum = zeros(num_unique_pts, 1);

        for seg_idx = 1:num_segs
            idx_start = (seg_idx - 1) * step + 1;
            idx_end   = idx_start + window_len - 1;
            if idx_end > N, break; end

            seg = x(idx_start:idx_end) .* w;
            X = fft(seg, nfft);

            % Two-sided periodogram
            P2 = (abs(X).^2) / (Fs * U);
            % Single-sided conversion
            P1 = P2(1:num_unique_pts);
            P1(2:end-1) = 2 * P1(2:end-1);

            pxx_accum = pxx_accum + P1;
        end

        pxx = pxx_accum / num_segs;
        f = (0:num_unique_pts-1)' * (Fs / nfft);
    end

    % Ensure column vectors
    pxx = pxx(:);
    f   = f(:);
end
