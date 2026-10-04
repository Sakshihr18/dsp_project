===============================================================================
CASE WESTERN RESERVE UNIVERSITY (CWRU) BEARING VIBRATION DATASET INSTRUCTIONS
===============================================================================

This directory is designated for the CWRU Bearing Dataset MAT-files (.mat).
Do NOT modify the source code to fit arbitrary file paths; simply place your
downloaded CWRU dataset files in this folder.

-------------------------------------------------------------------------------
1. DATASET OVERVIEW & DOWNLOAD
-------------------------------------------------------------------------------
The CWRU Bearing Data Center provides standard benchmark vibration signals:
URL: https://engineering.case.edu/bearingdatacenter

Data is categorized into four primary bearing operating conditions:
  1. Normal Baseline Data (Healthy Bearing)
  2. 12k Drive End Bearing Fault Data (Inner Race, Ball, Outer Race Faults)
  3. 48k Drive End Bearing Fault Data (Drive End accelerometer at 48,000 Hz)
  4. 12k Fan End Bearing Fault Data

Standard Motor Loads and Speeds:
  - 0 HP: approx. 1797 RPM (~29.95 Hz)
  - 1 HP: approx. 1772 RPM (~29.53 Hz)
  - 2 HP: approx. 1750 RPM (~29.17 Hz)
  - 3 HP: approx. 1730 RPM (~28.83 Hz)

-------------------------------------------------------------------------------
2. RECOMMENDED FILE PLACEMENT EXAMPLES
-------------------------------------------------------------------------------
Download the relevant .mat files and place them directly in this directory:

  - Normal (Healthy):
      dataset/97.mat       (48k Normal Baseline, 1797 RPM, 0 HP)
      dataset/98.mat       (48k Normal Baseline, 1772 RPM, 1 HP)

  - Inner Race Fault:
      dataset/105.mat      (12k DE, 0.007" IR Fault, 1797 RPM)
      dataset/209.mat      (12k DE, 0.014" IR Fault, 1797 RPM)

  - Ball Fault:
      dataset/118.mat      (12k DE, 0.007" Ball Fault, 1797 RPM)
      dataset/222.mat      (12k DE, 0.014" Ball Fault, 1797 RPM)

  - Outer Race Fault (Orthogonal @ 6:00 o'clock position):
      dataset/130.mat      (12k DE, 0.007" OR Fault, 1797 RPM)
      dataset/234.mat      (12k DE, 0.014" OR Fault, 1797 RPM)

-------------------------------------------------------------------------------
3. INTERNAL VARIABLE NAMING IN CWRU MAT-FILES
-------------------------------------------------------------------------------
Inside each CWRU .mat file, vibration signals are stored with variable names
reflecting the experiment number and sensor location, for example:
  - 'X097_DE_time' : Drive End accelerometer vibration signal (Default channel)
  - 'X097_FE_time' : Fan End accelerometer vibration signal
  - 'X097_BA_time' : Base accelerometer vibration signal
  - 'X097RPM'      : Motor speed in RPM

The loader function (source/load_signal.m) automatically detects and extracts
the Drive End (DE) acceleration signal, and safely accommodates variable name
variations without needing hard-coded field names.

-------------------------------------------------------------------------------
4. CONFIGURATION IN MAIN.M
-------------------------------------------------------------------------------
Before running source/main.m, ensure the top configuration section matches your
target file:
  - dataset_file = fullfile('..', 'dataset', '97.mat');  % Set target file
  - Fs           = 48000;   % 48000 for 48k recordings, or 12000 for 12k
  - shaft_rpm    = 1797;    % Match motor load (1797 for 0 HP, 1772 for 1 HP)

-------------------------------------------------------------------------------
5. DRIVE-END BEARING SPECIFICATION (SKF 6205-2RS JEM)
-------------------------------------------------------------------------------
  - Number of Rolling Elements (Nb) : 9
  - Ball Diameter (d)               : 0.3126 in (7.940 mm)
  - Pitch Diameter (D)              : 1.537 in (39.04 mm)
  - Contact Angle (theta)           : 0 degrees
===============================================================================
