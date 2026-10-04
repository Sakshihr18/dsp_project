function features = time_domain_features(signal)
% TIME_DOMAIN_FEATURES Calculate statistical time-domain features for DSP signal characterization.
%
% DSP THEORY & PHYSICAL INTERPRETATION:
%   Time-domain statistical parameters quantify the amplitude distribution and
%   impulsive nature of vibration waveforms without requiring machine learning:
%
%   1. RMS (Root Mean Square):
%      RMS = sqrt( (1/N) * sum( x(n)^2 ) )
%      Physical Meaning: Represents the effective vibration energy and power
%      content. Increases steadily as bearing surface wear and spalling worsen.
%
%   2. Peak Amplitude (Pk):
%      Peak = max( |x(n)| )
%      Physical Meaning: Captures the maximum instantaneous impact force
%      generated when a rolling element strikes a localized fatigue crack.
%
%   3. Peak-to-Peak (P2P):
%      PeakToPeak = max(x(n)) - min(x(n))
%      Physical Meaning: The total dynamic excursion of the housing vibration.
%
%   4. Kurtosis:
%      Kurtosis = (1/N) * sum( (x(n) - mu)^4 ) / (sigma^4)
%      Physical Meaning: The 4th standardized central moment measuring the
%      "tailedness" or impulsiveness of the probability density function (PDF).
%      - Healthy bearing (Gaussian distributed background noise): Kurtosis ~ 3.0
%      - Damaged bearing (sharp transient impacts): Kurtosis > 3.5 - 10.0+
%      Kurtosis is a primary DSP indicator of localized rolling element defects.
%
%   5. Crest Factor (CF):
%      CrestFactor = Peak / RMS
%      Physical Meaning: Ratio of peak shock level to effective continuous energy.
%      A high crest factor indicates early-stage localized pitting/spalling
%      before widespread continuous wear elevates the RMS floor.
%
%   6. Skewness:
%      Skewness = (1/N) * sum( (x(n) - mu)^3 ) / (sigma^3)
%      Physical Meaning: The 3rd standardized moment measuring waveform
%      asymmetry about the mean.
%
%   7. Supplementary Diagnostic Descriptors:
%      - Shape Factor   = RMS / mean(|x|)
%      - Impulse Factor = Peak / mean(|x|)
%      - Margin Factor  = Peak / (mean(sqrt(|x|)))^2
%
%   NOTE: These metrics are NOT fed into any ML model. They provide direct
%   DSP physical indicators used to validate rule-based envelope diagnosis.
%
% INPUTS:
%   signal   - 1D vibration signal (vector)
%
% OUTPUTS:
%   features - Struct with named physical features:
%                .RMS
%                .Peak
%                .PeakToPeak
%                .Kurtosis
%                .CrestFactor
%                .Skewness
%                .ShapeFactor
%                .ImpulseFactor
%                .MarginFactor
%
% Author: DSP Project Team
% Project: Bearing Fault Diagnosis Using Motor Vibration Signals

    x = double(signal(:));
    N = length(x);

    if N == 0
        error('DSP_TIME:EmptySignal', 'Input signal must not be empty.');
    end

    % Central moments and baseline values
    mu     = mean(x);
    abs_x  = abs(x);
    mean_abs = mean(abs_x);
    if mean_abs < 1e-12, mean_abs = 1e-12; end

    % Variance and standard deviation
    var_x  = mean((x - mu).^2);
    sigma  = sqrt(var_x);
    if sigma < 1e-12, sigma = 1e-12; end

    % 1. RMS
    rms_val = sqrt(mean(x.^2));
    if rms_val < 1e-12, rms_val = 1e-12; end

    % 2. Peak
    peak_val = max(abs_x);

    % 3. Peak-to-Peak
    p2p_val = max(x) - min(x);

    % 4. Kurtosis (4th standardized central moment)
    kurt_val = mean((x - mu).^4) / (sigma^4);

    % 5. Crest Factor
    crest_val = peak_val / rms_val;

    % 6. Skewness (3rd standardized central moment)
    skew_val = mean((x - mu).^3) / (sigma^3);

    % 7. Supplementary Factors
    shape_val   = rms_val / mean_abs;
    impulse_val = peak_val / mean_abs;
    margin_val  = peak_val / ((mean(sqrt(abs_x)))^2 + 1e-12);

    % Construct structured feature output
    features.RMS           = rms_val;
    features.Peak          = peak_val;
    features.PeakToPeak    = p2p_val;
    features.Kurtosis      = kurt_val;
    features.CrestFactor   = crest_val;
    features.Skewness      = skew_val;
    features.ShapeFactor   = shape_val;
    features.ImpulseFactor = impulse_val;
    features.MarginFactor  = margin_val;
end
