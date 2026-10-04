function [segments, time_indices, seg_info] = segmentation(signal, window_size, overlap_ratio)
% SEGMENTATION Divide continuous vibration signal into overlapping fixed-length windows.
%
% DSP THEORY & IMPLEMENTATION:
%   Mechanical vibration signals exhibit cyclo-stationary behavior where fault
%   transients occur periodically as the shaft rotates. Dividing long records
%   into short, stationary or quasi-stationary segments (e.g. 2048 samples)
%   enables:
%     1. Localized time-frequency spectral inspection.
%     2. Ensembling / averaging of spectral estimates across multiple defect impacts.
%     3. Memory efficiency and uniform window lengths for fast Radix-2 FFT computations.
%
%   Overlap (typically 50%):
%     Applying a tapered window (like Hann) attenuates data at the segment edges.
%     Overlapping segments by 50% ensures every sample receives equal total
%     weight across overlapping frames, preventing loss of transient impact data
%     occurring near the window boundaries.
%
%   Edge-case handling:
%     If the total signal length is shorter than `window_size`, the signal is
%     safely zero-padded to `window_size` and a single segment is returned with
%     an informative warning.
%
% INPUTS:
%   signal        - Input 1D vibration signal (column vector)
%   window_size   - Number of samples per segment (default: 2048)
%   overlap_ratio - Fraction of overlap between consecutive segments [0, 1) (default: 0.50)
%
% OUTPUTS:
%   segments      - Matrix of size [window_size x num_segments]
%   time_indices  - Matrix of size [num_segments x 2] indicating [start_sample, end_sample]
%   seg_info      - Struct containing segmentation summary metadata
%
% Author: DSP Project Team
% Project: Bearing Fault Diagnosis Using Motor Vibration Signals

    if nargin < 2 || isempty(window_size)
        window_size = 2048; % Standard project specification
    end
    if nargin < 3 || isempty(overlap_ratio)
        overlap_ratio = 0.50; % 50% overlap
    end

    signal_col = signal(:);
    total_samples = length(signal_col);

    % Validate parameters
    window_size = round(window_size);
    if window_size < 16
        error('DSP_SEG:InvalidWindowSize', 'window_size must be at least 16 samples.');
    end
    if overlap_ratio < 0 || overlap_ratio >= 1
        error('DSP_SEG:InvalidOverlap', 'overlap_ratio must satisfy 0 <= overlap_ratio < 1.');
    end

    step_size = round(window_size * (1 - overlap_ratio));
    if step_size < 1
        step_size = 1;
    end

    % Safe handling when input is shorter than one window
    if total_samples < window_size
        warning('DSP_SEG:SignalTooShort', ...
            'Signal length (%d) is shorter than window_size (%d). Zero-padding to %d samples.', ...
            total_samples, window_size, window_size);
        padded = zeros(window_size, 1);
        padded(1:total_samples) = signal_col;
        segments = padded;
        time_indices = [1, total_samples];
        num_segments = 1;
    else
        % Compute total number of complete segments
        num_segments = floor((total_samples - window_size) / step_size) + 1;
        segments = zeros(window_size, num_segments);
        time_indices = zeros(num_segments, 2);

        for i = 1:num_segments
            start_idx = (i - 1) * step_size + 1;
            end_idx   = start_idx + window_size - 1;
            segments(:, i) = signal_col(start_idx:end_idx);
            time_indices(i, :) = [start_idx, end_idx];
        end
    end

    % Summary metadata
    seg_info.num_segments   = num_segments;
    seg_info.window_size    = window_size;
    seg_info.step_size      = step_size;
    seg_info.overlap_ratio  = overlap_ratio;
    seg_info.total_samples  = total_samples;
    seg_info.first_segment  = segments(:, 1);

    fprintf('[segmentation] Created %d segments (Window: %d, Overlap: %.0f%%, Step: %d)\n', ...
        num_segments, window_size, overlap_ratio * 100, step_size);
end
