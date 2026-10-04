function [norm_signal, norm_params] = normalization(signal, method)
% NORMALIZATION Scale vibration signal for consistent segment-to-segment comparison while preserving physical metadata.
%
% DSP THEORY & JUSTIFICATION:
%   In machinery condition monitoring, vibration levels vary significantly
%   across operating loads, sensor sensitivities, and mounting conditions.
%   To perform meaningful spectral comparison and peak detection across
%   different segments or experimental runs, amplitude normalization is
%   applied.
%
%   Supported methods:
%     1. 'zscore' (Standardization, default):
%          x_norm = (x - mu) / sigma
%        Sets mean to 0 and variance to 1. In vibration analysis, sigma is
%        identically equal to the AC RMS value. This makes kurtosis and
%        spectral peak prominence thresholds dimensionless and invariant to
%        global sensor gain variations.
%     2. 'peak' (Full-scale peak scaling):
%          x_norm = x / max(|x|)
%        Constrains peak amplitudes strictly within [-1, +1].
%     3. 'rms' (Unit energy scaling):
%          x_norm = x / rms(x)
%        Normalizes total signal energy/power to 1.
%
%   Crucially, the original physical parameters (mu, sigma, peak, rms) are
%   retained in the output struct `norm_params` so physical units (e.g. g or m/s^2)
%   are never permanently lost.
%
% INPUTS:
%   signal      - Input 1D vibration signal (column vector)
%   method      - (Optional) Normalization method: 'zscore' (default), 'peak', 'rms'
%
% OUTPUTS:
%   norm_signal - Normalized vibration signal
%   norm_params - Struct containing statistical scaling metrics:
%                   .method
%                   .mean
%                   .std
%                   .peak
%                   .rms
%
% Author: DSP Project Team
% Project: Bearing Fault Diagnosis Using Motor Vibration Signals

    if nargin < 2 || isempty(method)
        method = 'zscore';
    end

    signal_col = signal(:);

    % Compute baseline statistics
    mu    = mean(signal_col);
    sigma = std(signal_col);
    pk    = max(abs(signal_col));
    r_val = sqrt(mean(signal_col.^2));

    % Avoid division by zero on silent/flat signals
    if sigma < 1e-12, sigma = 1e-12; end
    if pk < 1e-12,    pk = 1e-12;    end
    if r_val < 1e-12, r_val = 1e-12; end

    switch lower(method)
        case 'zscore'
            norm_signal = (signal_col - mu) / sigma;
        case 'peak'
            norm_signal = signal_col / pk;
        case 'rms'
            norm_signal = signal_col / r_val;
        case 'none'
            norm_signal = signal_col;
        otherwise
            warning('DSP_NORM:UnknownMethod', ...
                'Unknown normalization method "%s". Defaulting to zscore.', method);
            norm_signal = (signal_col - mu) / sigma;
            method = 'zscore';
    end

    % Record scaling parameters for physical reconstruction
    norm_params.method = method;
    norm_params.mean   = mu;
    norm_params.std    = sigma;
    norm_params.peak   = pk;
    norm_params.rms    = r_val;
end
