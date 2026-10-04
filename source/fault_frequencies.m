function char_freqs = fault_frequencies(geom_params, shaft_rpm, num_harmonics)
% FAULT_FREQUENCIES Calculate kinematic bearing characteristic fault frequencies from physical dimensions.
%
% KINEMATIC THEORY OF ROLLING ELEMENT BEARINGS:
%   When a rolling element bearing operates under load, defects on specific
%   components generate periodic impacts at characteristic frequencies dictated
%   by the internal kinematics of epicyclic planetary motion:
%
%   1. Shaft Rotational Frequency (fr):
%        fr = RPM / 60   [Hz]
%      Fundamental mechanical rotation frequency of the shaft and inner ring.
%
%   2. Fundamental Train Frequency (FTF):
%        FTF = (fr / 2) * (1 - (d / D) * cos(theta))
%      Represents the rotational speed of the cage assembly holding the balls.
%      Defects on the cage generate vibration at FTF.
%
%   3. Ball Pass Frequency Outer Race (BPFO):
%        BPFO = (Nb / 2) * fr * (1 - (d / D) * cos(theta)) = Nb * FTF
%      Rate at which rolling elements pass over a single fixed point on the
%      stationary outer raceway.
%
%   4. Ball Pass Frequency Inner Race (BPFI):
%        BPFI = (Nb / 2) * fr * (1 + (d / D) * cos(theta)) = Nb * (fr - FTF)
%      Rate at which rolling elements pass over a defect rotating on the inner
%      raceway. Because the inner race rotates through the mechanical load zone,
%      BPFI is amplitude-modulated by the shaft rotational frequency fr,
%      producing characteristic sidebands at (BPFI +/- k*fr).
%
%   5. Ball Spin Frequency (BSF):
%        BSF = (D / (2 * d)) * fr * (1 - ((d / D) * cos(theta))^2)
%      Rotational speed of an individual ball spinning on its own axis.
%      NOTE: As a defective ball spins, its defect impacts both the inner ring
%      and the outer ring during each 360-degree rotation. Therefore, the defect
%      impact rate is two impacts per ball revolution: 2*BSF (also known as
%      Ball Defect Frequency, BDF).
%
% INPUTS:
%   geom_params   - Struct containing bearing geometry:
%                     .Nb    - Number of rolling elements (balls/rollers)
%                     .d     - Ball diameter (mm or inches)
%                     .D     - Pitch diameter (same units as d)
%                     .theta - Contact angle in degrees (default: 0 for deep groove)
%   shaft_rpm     - Motor shaft rotational speed in RPM
%   num_harmonics - (Optional) Number of harmonic multiples to compute (default: 3)
%
% OUTPUTS:
%   char_freqs    - Struct containing:
%                     .fr        - Shaft rotational frequency [Hz]
%                     .FTF       - Fundamental Train Frequency [Hz]
%                     .BPFO      - Ball Pass Frequency Outer race [Hz]
%                     .BPFI      - Ball Pass Frequency Inner race [Hz]
%                     .BSF       - Ball Spin Frequency [Hz]
%                     .twoBSF    - 2 * BSF (Ball defect impact rate) [Hz]
%                     .harmonics - Struct containing harmonic vectors up to num_harmonics
%                     .descriptions - Text descriptions of each frequency
%                     .geom_used - Geometry parameters used
%
% Author: DSP Project Team
% Project: Bearing Fault Diagnosis Using Motor Vibration Signals

    if nargin < 3 || isempty(num_harmonics)
        num_harmonics = 3;
    end

    % Validate geometric parameters
    if ~isfield(geom_params, 'Nb') || isempty(geom_params.Nb)
        error('DSP_FREQ:MissingParam', 'geom_params.Nb (number of balls) is required.');
    end
    if ~isfield(geom_params, 'd') || isempty(geom_params.d)
        error('DSP_FREQ:MissingParam', 'geom_params.d (ball diameter) is required.');
    end
    if ~isfield(geom_params, 'D') || isempty(geom_params.D)
        error('DSP_FREQ:MissingParam', 'geom_params.D (pitch diameter) is required.');
    end
    if ~isfield(geom_params, 'theta') || isempty(geom_params.theta)
        geom_params.theta = 0; % Default to 0 degrees contact angle for deep-groove ball bearings
    end

    Nb    = double(geom_params.Nb);
    d     = double(geom_params.d);
    D     = double(geom_params.D);
    theta = double(geom_params.theta) * (pi / 180); % Convert degrees to radians

    % Geometry ratio
    gamma = (d / D) * cos(theta);

    if gamma >= 1
        error('DSP_FREQ:InvalidGeometry', 'Invalid bearing dimensions: (d/D)*cos(theta) must be < 1.');
    end

    % 1. Shaft Rotational Frequency
    fr = double(shaft_rpm) / 60;

    % 2. Kinematic Characteristic Frequencies
    FTF   = (fr / 2) * (1 - gamma);
    BPFO  = (Nb / 2) * fr * (1 - gamma);
    BPFI  = (Nb / 2) * fr * (1 + gamma);
    BSF   = (D / (2 * d)) * fr * (1 - gamma^2);
    twoBSF = 2 * BSF;

    % 3. Harmonic Series Construction
    harm_mult = (1:num_harmonics)';
    harmonics.BPFO   = BPFO * harm_mult;
    harmonics.BPFI   = BPFI * harm_mult;
    harmonics.BSF    = BSF * harm_mult;
    harmonics.twoBSF = twoBSF * harm_mult;
    harmonics.FTF    = FTF * harm_mult;
    harmonics.fr     = fr * harm_mult;

    % 4. Assemble Output Struct
    char_freqs.fr        = fr;
    char_freqs.FTF       = FTF;
    char_freqs.BPFO      = BPFO;
    char_freqs.BPFI      = BPFI;
    char_freqs.BSF       = BSF;
    char_freqs.twoBSF    = twoBSF;
    char_freqs.harmonics = harmonics;
    char_freqs.rpm       = shaft_rpm;
    char_freqs.geom_used = geom_params;

    % Theoretical descriptions
    char_freqs.descriptions.fr     = 'Shaft Rotational Frequency (RPM/60)';
    char_freqs.descriptions.FTF    = 'Fundamental Train Frequency (Cage rotational speed)';
    char_freqs.descriptions.BPFO   = 'Ball Pass Frequency Outer Race (Defect on outer raceway)';
    char_freqs.descriptions.BPFI   = 'Ball Pass Frequency Inner Race (Defect on inner raceway)';
    char_freqs.descriptions.BSF    = 'Ball Spin Frequency (Ball rotational speed about own axis)';
    char_freqs.descriptions.twoBSF = '2x BSF / Ball Defect Frequency (Impacts per ball revolution)';

    fprintf('[fault_frequencies] Shaft Speed: %.2f Hz (%.1f RPM)\n', fr, shaft_rpm);
    fprintf('                    FTF  : %6.2f Hz  |  BPFO: %6.2f Hz\n', FTF, BPFO);
    fprintf('                    BPFI : %6.2f Hz  |  BSF : %6.2f Hz (2xBSF: %6.2f Hz)\n', ...
        BPFI, BSF, twoBSF);
end
