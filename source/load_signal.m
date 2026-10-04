function [raw_signal, Fs_effective, channel_info] = load_signal(file_path, Fs_user, channel_pref)
% LOAD_SIGNAL Load vibration signal safely from a MATLAB .mat dataset file.
%
% DSP THEORY & IMPLEMENTATION:
%   In mechanical vibration monitoring and bearing diagnostics (specifically
%   the Case Western Reserve University (CWRU) benchmark), accelerometer data
%   is captured at the Drive End (DE), Fan End (FE), or Base Plate (BA).
%   Variable names in CWRU .mat files are not uniform; they vary according to
%   the experiment ID (e.g., 'X097_DE_time', 'X105_FE_time', 'X118_BA_time').
%   This function inspects the internal structures of the .mat container,
%   dynamically identifies the appropriate accelerometer channel, verifies
%   vector integrity, and extracts the time-series signal without hard-coding
%   internal variable identifiers.
%
% INPUTS:
%   file_path    - Full or relative path to the .mat file
%   Fs_user      - Configured sampling frequency in Hz (e.g., 48000 or 12000)
%   channel_pref - Preferred sensor channel: 'DE' (Drive End, default),
%                  'FE' (Fan End), 'BA' (Base Accelerometer), or 'AUTO'
%
% OUTPUTS:
%   raw_signal   - 1D column vector containing raw vibration acceleration samples
%   Fs_effective - Sampling frequency determined for subsequent DSP processing
%   channel_info - Struct containing metadata: variable name, length, duration,
%                  detected RPM (if available in file), and channel type.
%
% Author: DSP Project Team
% Project: Bearing Fault Diagnosis Using Motor Vibration Signals

    % Handle optional arguments
    if nargin < 2 || isempty(Fs_user)
        Fs_user = 48000; % Default to standard 48 kHz project specification
    end
    if nargin < 3 || isempty(channel_pref)
        channel_pref = 'DE'; % Default to Drive End accelerometer
    end

    % 1. Verify file existence with a clear, informative message
    if ~isfile(file_path)
        error('DSP_LOAD:FileNotFound', ...
            ['\n========================================================================\n' ...
             'ERROR: Dataset file not found!\n' ...
             'Path checked: %s\n\n' ...
             'Please place the CWRU .mat file inside the dataset folder and update\n' ...
             'the file path configuration in main.m.\n' ...
             'Refer to dataset/README.txt for dataset download and placement instructions.\n' ...
             '========================================================================\n'], ...
             file_path);
    end

    % 2. Load the .mat file into a structured workspace
    try
        mat_contents = load(file_path);
    catch ME
        error('DSP_LOAD:CorruptFile', ...
            'Failed to read .mat file: %s. File may be corrupted or not a valid MAT-file.\nError: %s', ...
            file_path, ME.message);
    end

    var_names = fieldnames(mat_contents);
    if isempty(var_names)
        error('DSP_LOAD:EmptyMatFile', 'The loaded file %s contains no variables.', file_path);
    end

    % 3. Search for the preferred vibration channel
    target_var = '';
    channel_pref_upper = upper(channel_pref);

    % First pass: exact or substring match for preferred channel (e.g. 'DE_time' or '_DE_')
    for i = 1:length(var_names)
        vname = var_names{i};
        vname_upper = upper(vname);
        val = mat_contents.(vname);

        % Must be a numeric vector with significant length (> 100 samples)
        if isnumeric(val) && (isvector(val) || min(size(val)) == 1) && numel(val) > 100
            if strcmp(channel_pref_upper, 'AUTO')
                % In AUTO mode, prioritize DE if available
                if contains(vname_upper, 'DE')
                    target_var = vname;
                    break;
                end
            else
                if contains(vname_upper, channel_pref_upper)
                    target_var = vname;
                    break;
                end
            end
        end
    end

    % Second pass: if preferred channel was not found, find any vector containing 'time'
    if isempty(target_var)
        for i = 1:length(var_names)
            vname = var_names{i};
            vname_upper = upper(vname);
            val = mat_contents.(vname);
            if isnumeric(val) && isvector(val) && numel(val) > 100 && contains(vname_upper, 'TIME')
                target_var = vname;
                break;
            end
        end
    end

    % Third pass fallback: select the longest 1D numeric array in the file
    if isempty(target_var)
        max_len = 0;
        for i = 1:length(var_names)
            vname = var_names{i};
            val = mat_contents.(vname);
            if isnumeric(val) && isvector(val) && numel(val) > max_len
                max_len = numel(val);
                target_var = vname;
            end
        end
    end

    if isempty(target_var)
        error('DSP_LOAD:SignalNotFound', ...
            'Could not find a valid 1D vibration signal in file %s. Found variables: %s', ...
            file_path, strjoin(var_names, ', '));
    end

    % 4. Extract and shape signal as a column vector
    raw_signal = mat_contents.(target_var);
    raw_signal = double(raw_signal(:)); % Ensure double precision column vector

    % 5. Check if RPM variable is stored alongside in CWRU file
    detected_rpm = NaN;
    for i = 1:length(var_names)
        vname = var_names{i};
        if contains(upper(vname), 'RPM')
            rpm_val = mat_contents.(vname);
            if isnumeric(rpm_val) && isscalar(rpm_val)
                detected_rpm = double(rpm_val);
            end
        end
    end

    % 6. Construct metadata struct
    Fs_effective = Fs_user;
    channel_info.variable_name = target_var;
    channel_info.num_samples   = length(raw_signal);
    channel_info.duration_sec  = length(raw_signal) / Fs_effective;
    channel_info.detected_rpm  = detected_rpm;
    channel_info.channel_type  = channel_pref;
    channel_info.file_path     = file_path;

    fprintf('[load_signal] Successfully loaded: %s\n', file_path);
    fprintf('              Selected variable: %s (%d samples, %.3f s at %d Hz)\n', ...
        target_var, length(raw_signal), channel_info.duration_sec, Fs_effective);
    if ~isnan(detected_rpm)
        fprintf('              Detected RPM in file: %.1f RPM\n', detected_rpm);
    end
end
