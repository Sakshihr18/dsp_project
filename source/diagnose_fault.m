function diagnosis_report = diagnose_fault(peak_results, time_features, env_spectrum, f_env, char_freqs, diag_cfg)
% DIAGNOSE_FAULT Rule-based DSP expert diagnostic system for rolling element bearing condition assessment.
%
% DSP THEORY & SCIENTIFIC REASONING (ZERO MACHINE LEARNING):
%   Unlike opaque Machine Learning "black boxes" (SVM, Random Forests, Neural Nets),
%   this diagnostic engine is strictly deterministic and rooted in the physical
%   acoustics and kinematics of bearing failure modes:
%
%   1. Outer Race Fault (BPFO):
%      - Physical phenomenon: Stationary outer raceway defect. As each rolling
%        element passes over the spall, an impact occurs at exact frequency BPFO.
%      - Envelope signature: Sharp, unmodulated peaks at BPFO, 2*BPFO, 3*BPFO.
%        Because the outer ring is fixed relative to the stationary load zone,
%        BPFO produces minimal shaft rotational sidebands.
%
%   2. Inner Race Fault (BPFI):
%      - Physical phenomenon: Defect rotates with the shaft. It passes through
%        the heavy load zone (maximum impact) and then through the unloaded zone
%        (weaker or zero impact).
%      - Envelope signature: Sharp peaks at BPFI and its harmonics, AMPLITUDE-
%        MODULATED at the shaft rotational frequency fr. This creates distinct
%        sideband pairs at (BPFI +/- fr) and (BPFI +/- 2*fr).
%
%   3. Ball Fault (BSF / 2*BSF):
%      - Physical phenomenon: A damaged ball spins on its axis and impacts both
%        the inner and outer raceways, producing impacts at 2*BSF (and BSF).
%      - Envelope signature: Significant energy at 2*BSF and BSF, accompanied by
%        cage modulation sidebands at (BSF +/- FTF).
%
%   4. Healthy Bearing:
%      - Physical phenomenon: Smooth raceways and rolling elements. No periodic
%        shock excitations.
%      - Envelope signature: Flat noise floor with absence of characteristic
%        fault combs. Time-domain Kurtosis remains near Gaussian (~3.0) and
%        Crest Factor remains low (< 3.0).
%
%   MULTI-FACTOR EVIDENCE SCORING ARCHITECTURE (Max 100 points per fault type):
%     Factor A: Fundamental Peak Prominence vs Spectral Noise Floor (0 - 40 pts)
%     Factor B: Harmonic Comb Confirmation (2X, 3X multiples)       (0 - 30 pts)
%     Factor C: Narrowband Spectral Energy Concentration            (0 - 15 pts)
%     Factor D: Time-Domain Impulsiveness Corroboration (Kurtosis)  (0 - 15 pts)
%
% INPUTS:
%   peak_results  - Output struct from peak_detection.m
%   time_features - Output struct from time_domain_features.m
%   env_spectrum  - Envelope magnitude spectrum
%   f_env         - Envelope frequency vector in Hz
%   char_freqs    - Bearing characteristic frequencies struct
%   diag_cfg      - (Optional) Diagnostic configuration parameters
%
% OUTPUTS:
%   diagnosis_report - Struct containing:
%                        .FinalDiagnosis    - Diagnostic decision string
%                        .ConfidencePercent - Confidence rating [0 - 100%]
%                        .EvidenceScores    - Struct of numerical scores (BPFO, BPFI, BSF, FTF, Healthy)
%                        .SupportingEvidence- Cell array of explanatory physical rationales
%                        .Severity          - Qualitative severity indicator
%
% Author: DSP Project Team
% Project: Bearing Fault Diagnosis Using Motor Vibration Signals

    if nargin < 6 || isempty(diag_cfg)
        diag_cfg = struct();
    end

    if ~isfield(diag_cfg, 'fault_threshold') || isempty(diag_cfg.fault_threshold)
        diag_cfg.fault_threshold = 30.0; % Evidence score threshold to declare a fault
    end
    if ~isfield(diag_cfg, 'healthy_kurtosis_limit') || isempty(diag_cfg.healthy_kurtosis_limit)
        diag_cfg.healthy_kurtosis_limit = 3.6; % Gaussian noise upper bound
    end

    % 1. Evaluate BPFO Evidence Score (Outer Race)
    [score_BPFO, bpfo_reasons] = evaluate_harmonic_evidence( ...
        peak_results.matched_BPFO, env_spectrum, f_env, char_freqs.BPFO, ...
        time_features, 'Outer Race (BPFO)', false, char_freqs.fr);

    % 2. Evaluate BPFI Evidence Score (Inner Race - with shaft sidebands check)
    [score_BPFI, bpfi_reasons] = evaluate_harmonic_evidence( ...
        peak_results.matched_BPFI, env_spectrum, f_env, char_freqs.BPFI, ...
        time_features, 'Inner Race (BPFI)', true, char_freqs.fr);

    % 3. Evaluate BSF Evidence Score (Ball Fault)
    [score_BSF, bsf_reasons] = evaluate_ball_evidence( ...
        peak_results.matched_BSF, env_spectrum, f_env, char_freqs, time_features);

    % 4. Evaluate FTF Evidence Score (Cage Fault)
    score_FTF = evaluate_ftf_evidence(peak_results.matched_FTF, env_spectrum, f_env, char_freqs.FTF);

    % 5. Evaluate Healthy Evidence Score
    % A bearing is healthy when fault scores are suppressed and kurtosis is near Gaussian
    max_fault_score = max([score_BPFO, score_BPFI, score_BSF]);
    kurt = time_features.Kurtosis;
    cf   = time_features.CrestFactor;

    score_Healthy = 0;
    healthy_reasons = {};

    if max_fault_score < diag_cfg.fault_threshold
        score_Healthy = score_Healthy + 50 * (1 - (max_fault_score / diag_cfg.fault_threshold));
        healthy_reasons{end+1} = sprintf('No significant harmonic combs detected at BPFO, BPFI, or BSF.');
    end

    if kurt <= diag_cfg.healthy_kurtosis_limit
        score_Healthy = score_Healthy + 30 * max(0, (1 - (kurt - 3.0) / 1.5));
        healthy_reasons{end+1} = sprintf('Kurtosis (%.2f) aligns with Gaussian background vibration (<= %.1f).', ...
            kurt, diag_cfg.healthy_kurtosis_limit);
    end

    if cf <= 3.2
        score_Healthy = score_Healthy + 20 * max(0, (1 - (cf - 2.0) / 1.5));
        healthy_reasons{end+1} = sprintf('Crest factor (%.2f) shows absence of impulsive impacts (<= 3.2).', cf);
    end
    score_Healthy = min(100, max(0, score_Healthy));

    % 6. Rule-Based Decision Engine
    evidence_scores.BPFO    = score_BPFO;
    evidence_scores.BPFI    = score_BPFI;
    evidence_scores.BSF     = score_BSF;
    evidence_scores.FTF     = score_FTF;
    evidence_scores.Healthy = score_Healthy;

    final_diag   = 'Healthy Bearing';
    confidence   = score_Healthy;
    final_reasons = healthy_reasons;
    severity     = 'Normal';

    % Compare fault scores against threshold
    fault_scores = [score_BPFO, score_BPFI, score_BSF];
    [top_fault_score, top_idx] = max(fault_scores);

    if top_fault_score >= diag_cfg.fault_threshold
        switch top_idx
            case 1 % BPFO
                final_diag   = 'Outer Race Fault';
                confidence   = score_BPFO;
                final_reasons = bpfo_reasons;
            case 2 % BPFI
                final_diag   = 'Inner Race Fault';
                confidence   = score_BPFI;
                final_reasons = bpfi_reasons;
            case 3 % BSF
                final_diag   = 'Ball Fault';
                confidence   = score_BSF;
                final_reasons = bsf_reasons;
        end

        % Estimate Severity from Kurtosis and Harmonics
        if kurt > 6.0 || confidence > 80
            severity = 'Severe';
        elseif kurt > 3.8 || confidence > 55
            severity = 'Moderate';
        else
            severity = 'Early / Incipient';
        end
    else
        % If fault scores are below threshold, confirm Healthy or mark Uncertain
        if score_Healthy >= 50
            final_diag = 'Healthy Bearing';
            confidence = score_Healthy;
            severity   = 'Normal';
        else
            % Ambiguous internal state: low fault evidence but elevated kurtosis/noise
            final_diag = 'Healthy Bearing'; % Safe baseline map
            confidence = max(40, score_Healthy);
            final_reasons{end+1} = 'Note: Vibration is mildly perturbed, but characteristic fault patterns are absent.';
            severity   = 'Indeterminate / Minor Anomaly';
        end
    end

    % 7. Construct Report Struct
    diagnosis_report.FinalDiagnosis     = final_diag;
    diagnosis_report.ConfidencePercent  = round(confidence, 1);
    diagnosis_report.EvidenceScores     = evidence_scores;
    diagnosis_report.SupportingEvidence = final_reasons;
    diagnosis_report.Severity           = severity;
    diagnosis_report.TimeFeatures       = time_features;
    diagnosis_report.CharFreqs          = char_freqs;
end

% =========================================================================
% DSP SCORING UTILITIES
% =========================================================================

function [score, reasons] = evaluate_harmonic_evidence(matched_harmonics, env_spec, f_env, f_target, time_feat, name, check_sidebands, fr)
    score = 0;
    reasons = {};

    % Factor A: Fundamental Peak Prominence (0 to 40 pts)
    fund = matched_harmonics(1);
    if fund.is_detected
        prom_snr = fund.prom_to_noise;
        pts_fund = min(40, 10 + 6 * log2(max(1, prom_snr)));
        score = score + pts_fund;
        reasons{end+1} = sprintf('Dominant peak detected at %s fundamental (%.2f Hz, Prominence/Noise = %.1fx, +%.1f pts).', ...
            name, fund.detected_freq, prom_snr, pts_fund);
    end

    % Factor B: Harmonic Comb Multiples (2X, 3X) (0 to 30 pts)
    num_harm_detected = sum([matched_harmonics.is_detected]);
    if num_harm_detected >= 2
        pts_harm = min(30, (num_harm_detected - 1) * 15);
        score = score + pts_harm;
        reasons{end+1} = sprintf('Harmonic comb confirmed: %d of %d harmonics detected (+%.1f pts).', ...
            num_harm_detected, length(matched_harmonics), pts_harm);
    end

    % Factor C: Narrowband Spectral Energy (0 to 15 pts)
    energy_ratio = compute_band_energy_ratio(env_spec, f_env, f_target, length(matched_harmonics));
    if energy_ratio > 0.05
        pts_nrg = min(15, energy_ratio * 100);
        score = score + pts_nrg;
        reasons{end+1} = sprintf('Substantial envelope spectral energy (%.1f%%) concentrated at %s harmonics (+%.1f pts).', ...
            energy_ratio * 100, name, pts_nrg);
    end

    % Factor D: Time-Domain Impulsiveness Corroboration (0 to 15 pts)
    if time_feat.Kurtosis > 3.5
        pts_kurt = min(15, (time_feat.Kurtosis - 3.0) * 3);
        score = score + pts_kurt;
        reasons{end+1} = sprintf('Elevated Kurtosis (%.2f) confirms impulsive shocks from surface fatigue (+%.1f pts).', ...
            time_feat.Kurtosis, pts_kurt);
    end

    % Inner race bonus: Shaft modulation sidebands check (BPFI +/- fr)
    if check_sidebands && fund.is_detected
        sb_detected = check_modulation_sidebands(env_spec, f_env, fund.detected_freq, fr);
        if sb_detected
            score = score + 10;
            reasons{end+1} = sprintf('Shaft modulation sidebands (BPFI +/- fr = %.2f +/- %.2f Hz) detected, confirming rotating inner raceway defect (+10 pts).', ...
                fund.detected_freq, fr);
        end
    end

    score = min(100, max(0, score));
end

function [score, reasons] = evaluate_ball_evidence(matched_ball, env_spec, f_env, char_freqs, time_feat)
    score = 0;
    reasons = {};

    m_bsf  = matched_ball.BSF_harmonics;
    m_2bsf = matched_ball.twoBSF_harmonics;

    % Factor A: 2*BSF or BSF Fundamental Detection
    fund_2bsf_det = m_2bsf(1).is_detected;
    fund_bsf_det  = m_bsf(1).is_detected;

    if fund_2bsf_det
        prom_snr = m_2bsf(1).prom_to_noise;
        pts_fund = min(40, 15 + 6 * log2(max(1, prom_snr)));
        score = score + pts_fund;
        reasons{end+1} = sprintf('Peak detected at Ball Defect Frequency 2*BSF (%.2f Hz, Prominence/Noise = %.1fx, +%.1f pts).', ...
            m_2bsf(1).detected_freq, prom_snr, pts_fund);
    elseif fund_bsf_det
        prom_snr = m_bsf(1).prom_to_noise;
        pts_fund = min(30, 10 + 5 * log2(max(1, prom_snr)));
        score = score + pts_fund;
        reasons{end+1} = sprintf('Peak detected at Ball Spin Frequency BSF (%.2f Hz, Prominence/Noise = %.1fx, +%.1f pts).', ...
            m_bsf(1).detected_freq, prom_snr, pts_fund);
    end

    % Factor B: Harmonics of 2*BSF and BSF
    total_ball_harm = sum([m_bsf.is_detected]) + sum([m_2bsf.is_detected]);
    if total_ball_harm >= 2
        pts_harm = min(30, total_ball_harm * 10);
        score = score + pts_harm;
        reasons{end+1} = sprintf('Ball defect harmonic activity confirmed (%d peaks detected, +%.1f pts).', ...
            total_ball_harm, pts_harm);
    end

    % Factor C: Spectral Energy
    energy_ratio = compute_band_energy_ratio(env_spec, f_env, char_freqs.twoBSF, 2);
    if energy_ratio > 0.04
        pts_nrg = min(15, energy_ratio * 120);
        score = score + pts_nrg;
        reasons{end+1} = sprintf('Envelope energy concentrated near ball spin harmonics (%.1f%%, +%.1f pts).', ...
            energy_ratio * 100, pts_nrg);
    end

    % Factor D: Time-Domain Impulsiveness
    if time_feat.Kurtosis > 3.5
        pts_kurt = min(15, (time_feat.Kurtosis - 3.0) * 3);
        score = score + pts_kurt;
    end

    score = min(100, max(0, score));
end

function score = evaluate_ftf_evidence(matched_ftf, env_spec, f_env, ftf)
    score = 0;
    if matched_ftf(1).is_detected
        score = score + 25;
    end
    energy_ratio = compute_band_energy_ratio(env_spec, f_env, ftf, 1);
    score = score + min(25, energy_ratio * 150);
    score = min(100, max(0, score));
end

function ratio = compute_band_energy_ratio(env_spec, f_env, f_target, K)
% Computes fraction of envelope spectral power within +/- 2.5 Hz bands around harmonics
    total_power = sum(env_spec.^2);
    if total_power < 1e-12, ratio = 0; return; end

    band_power = 0;
    delta_f = 2.5; % Hz
    for k = 1:K
        fk = k * f_target;
        in_band = abs(f_env - fk) <= delta_f;
        band_power = band_power + sum(env_spec(in_band).^2);
    end
    ratio = band_power / total_power;
end

function detected = check_modulation_sidebands(env_spec, f_env, f_carrier, fr)
% Checks for modulation sidebands at f_carrier +/- fr
    delta_f = 2.5;
    sb_left_band  = abs(f_env - (f_carrier - fr)) <= delta_f;
    sb_right_band = abs(f_env - (f_carrier + fr)) <= delta_f;

    noise = median(env_spec);
    left_pk  = max(env_spec(sb_left_band));
    right_pk = max(env_spec(sb_right_band));

    detected = (~isempty(left_pk) && left_pk > 2.0 * noise) || ...
               (~isempty(right_pk) && right_pk > 2.0 * noise);
end
