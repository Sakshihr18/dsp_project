function [envelope_sig, env_spectrum, f_env] = envelope_analysis(filtered_signal, Fs, apply_hanning)
% ENVELOPE_ANALYSIS Demodulate high-frequency resonance using the Hilbert transform to extract the envelope spectrum.
%
% DSP THEORY & PRINCIPLE OF ENVELOPE DEMODULATION (HIGH FREQUENCY RESONANCE TECHNIQUE):
%   1. Physical Mechanism:
%      When a rolling element rolls across a localized crack or spall on an
%      inner race, outer race, or ball, an abrupt mechanical shock is generated.
%      This impulse acts like a Dirac delta input, exciting the high-frequency
%      structural resonant modes of the bearing outer ring and sensor assembly
%      (carrier band, e.g. 2-8 kHz).
%
%   2. Why Direct FFT Fails for Early Faults:
%      In a raw FFT, the energy of these short impulses is smeared across the
%      entire spectrum and drowned out by dominant low-frequency mechanical
%      sources (1X unbalance, 2X misalignment, 50/60 Hz electrical hum).
%
%   3. Hilbert Demodulation Pipeline:
%      a) Band-pass filtering isolates the resonance carrier band.
%      b) Hilbert Transform computes the quadrature 90-degree phase-shifted signal:
%           x_hat(t) = H{ x(t) } = (1/pi) * p.v. \int [ x(tau) / (t - tau) ] dtau
%      c) Analytic Signal formation:
%           z(t) = x(t) + j * x_hat(t)
%      d) Envelope extraction (instantaneous amplitude):
%           A(t) = |z(t)| = sqrt( x(t)^2 + x_hat(t)^2 )
%      e) DC offset removal:
%           A_ac(t) = A(t) - mean(A(t))
%         Removing the mean envelope eliminates a massive 0 Hz artifact that would
%         otherwise mask low-frequency fault frequencies (10 Hz - 300 Hz).
%      f) Hann Windowing:
%         Mitigates spectral leakage on the demodulated envelope.
%      g) FFT of Envelope:
%         Demodulates the high-frequency resonance down to baseband, displaying
%         clear harmonic combs at BPFO, BPFI, BSF, and FTF.
%
% INPUTS:
%   filtered_signal - Band-pass filtered vibration signal (resonance band)
%   Fs              - Sampling frequency in Hz
%   apply_hanning   - (Optional) Boolean flag to apply Hann window to envelope (default: true)
%
% OUTPUTS:
%   envelope_sig - Extracted time-domain envelope amplitude A(t)
%   env_spectrum - Scaled single-sided magnitude spectrum of the detrended envelope
%   f_env        - Frequency vector for envelope spectrum in Hz
%
% Author: DSP Project Team
% Project: Bearing Fault Diagnosis Using Motor Vibration Signals

    if nargin < 3 || isempty(apply_hanning)
        apply_hanning = true;
end

    x = double(filtered_signal( :));
N = length(x);

if N
  < 16 error('DSP_ENV:SignalTooShort',
             'Signal length must be >= 16 samples for envelope analysis.');
end

    % 1. Compute Analytic Signal via Hilbert Transform try %
    Signal Processing Toolbox hilbert z = hilbert(x);
    catch
        % Fallback implementation: Exact frequency-domain Hilbert transform
        %   z(n) = IFFT{ X(k) .* H(k) }
        %   where H(k) = 1 for k=0, Nyquist; H(k) = 2 for positive freqs; H(k) = 0 for negative freqs
        X = fft(x);
    H = zeros(N, 1);
    if rem (N, 2)
      == 0 % Even length H(1) = 1;
    H(N / 2 + 1) = 1;
    H(2 : N / 2) = 2;
    else % Odd length H(1) = 1;
    H(2 : (N + 1) / 2) = 2;
    end z = ifft(X.*H);
    end

        % 2. Extract Envelope(Instantaneous Amplitude) envelope_sig = abs(z);

    % 3. Remove DC Component(Eliminates 0 Hz spike in spectrum) env_ac =
        envelope_sig - mean(envelope_sig);

    % 4. Apply Hanning Window to Envelope Signal
    if apply_hanning
        w = 0.5 * (1 - cos(2 * pi * (0:N-1)' / (N - 1)));
        env_win = env_ac .* w;
        coherent_gain = mean(w);
    else
        env_win = env_ac;
        coherent_gain = 1.0;
    end

    % 5. Compute Single-Sided FFT of Demodulated Envelope
    Y_env = fft(env_win);
    num_bins = floor(N / 2) + 1;

    P2 = abs(Y_env / N);
    P1 = P2(1:num_bins);
    P1(2:end-1) = 2 * P1(2:end-1); % Energy conservation

    env_spectrum = P1 / coherent_gain;
    f_env = (0:num_bins-1)' * (Fs / N);

    % Ensure column vectors
    envelope_sig = envelope_sig(:);
    env_spectrum = env_spectrum(:);
    f_env        = f_env(:);
end
