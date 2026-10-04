function [f, mag_spectrum, dominant_peaks] = fft_analysis(signal, Fs, apply_window, f_limit)
% FFT_ANALYSIS Single-sided discrete Fourier transform with Hanning windowing and proper amplitude scaling.
%
% DSP THEORY & SPECTRAL LEAKAGE MITIGATION:
%   Computing the Discrete Fourier Transform (DFT) via the Fast Fourier Transform
%   (FFT) assumes the finite-length observation window is periodically repeated
%   for all time: -inf < t < +inf.
%
%   If the segment length is not an exact integer multiple of all frequency
%   components present, a sharp discontinuity occurs between the last sample
%   and the repeated first sample. This abrupt jump acts like a step function,
%   spreading energy into adjacent spectral bins - an artifact known as
%   SPECTRAL LEAKAGE.
%
%   To mitigate spectral leakage:
%     1. A Hanning (Hann) window w(n) = 0.5 * (1 - cos(2*pi*n / (N-1))) is applied.
%        The Hanning window smoothly tapers the segment boundaries to zero,
%        eliminating endpoint discontinuity and reducing sidelobes to -31 dB.
%     2. Window Amplitude Correction:
%        Tapering reduces overall signal power. To preserve true physical peak
%        amplitudes (Coherent Gain correction), the spectrum is scaled by:
%        Amplitude_Correction_Factor = 1 / mean(w).
%     3. Single-Sided Scaling:
%        The negative frequencies (-f) are folded onto positive frequencies (+f)
%        by multiplying AC bins (k = 2 to N/2) by a factor of 2.
%
% INPUTS:
%   signal       - 1D vibration signal vector (e.g. 2048-sample segment)
%   Fs           - Sampling frequency in Hz
%   apply_window - (Optional) Boolean flag to apply Hanning window (default: true)
%   f_limit      - (Optional) Maximum frequency to retain in output (Hz, default: Fs/2)
%
% OUTPUTS:
%   f              - Frequency vector in Hz [0, f_limit]
%   mag_spectrum   - Scaled single-sided magnitude spectrum (same units as input signal)
%   dominant_peaks - Struct array of dominant peaks: .frequency, .magnitude
%
% Author: DSP Project Team
% Project: Bearing Fault Diagnosis Using Motor Vibration Signals

    if nargin < 3 || isempty(apply_window)
        apply_window = true;
    end

    x = double(signal(:));
    N = length(x);

    if nargin < 4 || isempty(f_limit)
        f_limit = Fs / 2;
    end

    % 1. Hanning Window Construction & Application
    if apply_window
        % Hanning window: w(n) = 0.5 * (1 - cos(2*pi*(0:N-1)' / (N - 1)))
        n_idx = (0:N-1)';
        w = 0.5 * (1 - cos(2 * pi * n_idx / (N - 1)));
        x_win = x .* w;
        % Coherent gain correction factor so peak amplitude of a pure sine is exact
        coherent_gain = mean(w);
    else
        x_win = x;
        coherent_gain = 1.0;
    end

    % 2. Fast Fourier Transform
    % Radix-2 / Bluestein FFT algorithm
    X_fft = fft(x_win);

    % 3. Scaled Single-Sided Spectrum Computation
    % Number of unique frequency bins up to Nyquist
    num_bins = floor(N / 2) + 1;
    P2 = abs(X_fft / N);              % Two-sided magnitude normalized by N
    P1 = P2(1:num_bins);             % Take positive frequencies
    P1(2:end-1) = 2 * P1(2:end-1);    % Conserve energy: double AC components

    % Apply coherent gain correction for window attenuation
    mag_full = P1 / coherent_gain;

    % Frequency vector
    f_full = (0:num_bins-1)' * (Fs / N);

    % 4. Constrain to f_limit if requested
    valid_idx = (f_full <= f_limit);
    f = f_full(valid_idx);
    mag_spectrum = mag_full(valid_idx);

    % 5. Identify Dominant Spectral Peaks (Top 5 largest peaks)
    dominant_peaks = struct('frequency', {}, 'magnitude', {});
    if length(mag_spectrum) > 3
        % Simple local maxima detection
        is_peak = false(length(mag_spectrum), 1);
        for k = 2:length(mag_spectrum)-1
            if mag_spectrum(k) > mag_spectrum(k-1) && mag_spectrum(k) > mag_spectrum(k+1)
                is_peak(k) = true;
            end
        end

        peak_freqs = f(is_peak);
        peak_mags  = mag_spectrum(is_peak);

        % Sort in descending order of magnitude
        [sorted_mags, sort_idx] = sort(peak_mags, 'descend');
        sorted_freqs = peak_freqs(sort_idx);

        num_top = min(5, length(sorted_mags));
        for p = 1:num_top
            dominant_peaks(p).frequency = sorted_freqs(p);
            dominant_peaks(p).magnitude = sorted_mags(p);
        end
    end
end
