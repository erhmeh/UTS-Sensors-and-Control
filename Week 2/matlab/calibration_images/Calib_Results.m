% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 692.293315624466572 ; 690.276326983612535 ];

%-- Principal point:
cc = [ 288.934595186064143 ; 241.580424490390357 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.085565718701942 ; -0.226506583612115 ; -0.004271036143178 ; 0.001062381996300 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 2.395624940435797 ; 2.382407922321477 ];

%-- Principal point uncertainty:
cc_error = [ 3.967999340352866 ; 3.195505613962954 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.014286053151786 ; 0.048436851477499 ; 0.001718403169213 ; 0.002256598742747 ; 0.000000000000000 ];

%-- Image size:
nx = 640;
ny = 480;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 10;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 2.071787e+00 ; 2.161826e+00 ; -1.575578e-01 ];
Tc_1  = [ -1.020675e+02 ; -1.172048e+02 ; 3.945602e+02 ];
omc_error_1 = [ 3.827145e-03 ; 4.312467e-03 ; 8.428141e-03 ];
Tc_error_1  = [ 2.291801e+00 ; 1.824871e+00 ; 1.695922e+00 ];

%-- Image #2:
omc_2 = [ 2.168762e+00 ; 2.241524e+00 ; 1.710400e-01 ];
Tc_2  = [ -1.197403e+02 ; -8.489394e+01 ; 4.317004e+02 ];
omc_error_2 = [ 4.961977e-03 ; 4.411234e-03 ; 1.014061e-02 ];
Tc_error_2  = [ 2.509598e+00 ; 2.038785e+00 ; 1.972673e+00 ];

%-- Image #3:
omc_3 = [ -1.960237e+00 ; -2.139606e+00 ; 4.149023e-01 ];
Tc_3  = [ -9.117615e+01 ; -1.182231e+02 ; 4.446366e+02 ];
omc_error_3 = [ 4.330155e-03 ; 4.178432e-03 ; 7.914604e-03 ];
Tc_error_3  = [ 2.551910e+00 ; 2.042728e+00 ; 1.727597e+00 ];

%-- Image #4:
omc_4 = [ 1.851097e+00 ; 1.792758e+00 ; -6.208276e-01 ];
Tc_4  = [ -1.261584e+02 ; -6.460569e+01 ; 4.927893e+02 ];
omc_error_4 = [ 3.441892e-03 ; 4.706473e-03 ; 7.090072e-03 ];
Tc_error_4  = [ 2.814025e+00 ; 2.282320e+00 ; 1.804112e+00 ];

%-- Image #5:
omc_5 = [ 1.975137e+00 ; 1.696808e+00 ; 2.143441e-01 ];
Tc_5  = [ -8.947519e+01 ; -1.055468e+02 ; 4.173972e+02 ];
omc_error_5 = [ 4.539672e-03 ; 4.015149e-03 ; 7.305502e-03 ];
Tc_error_5  = [ 2.429723e+00 ; 1.923762e+00 ; 1.840913e+00 ];

%-- Image #6:
omc_6 = [ -1.844129e+00 ; -1.909551e+00 ; -4.671595e-01 ];
Tc_6  = [ -1.014466e+02 ; -1.109879e+02 ; 3.512072e+02 ];
omc_error_6 = [ 3.147057e-03 ; 4.778137e-03 ; 7.487577e-03 ];
Tc_error_6  = [ 2.062459e+00 ; 1.659483e+00 ; 1.686261e+00 ];

%-- Image #7:
omc_7 = [ 1.626158e+00 ; 1.506413e+00 ; -1.353786e-02 ];
Tc_7  = [ -1.418103e+02 ; -9.237225e+01 ; 4.672995e+02 ];
omc_error_7 = [ 4.028403e-03 ; 4.674186e-03 ; 6.060881e-03 ];
Tc_error_7  = [ 2.706124e+00 ; 2.184373e+00 ; 2.041841e+00 ];

%-- Image #8:
omc_8 = [ 1.776633e+00 ; 2.180665e+00 ; -1.194178e+00 ];
Tc_8  = [ -9.273256e+01 ; -1.121117e+02 ; 5.488006e+02 ];
omc_error_8 = [ 2.782286e-03 ; 5.724356e-03 ; 7.998968e-03 ];
Tc_error_8  = [ 3.159184e+00 ; 2.556244e+00 ; 1.756014e+00 ];

%-- Image #9:
omc_9 = [ -1.346334e+00 ; -1.824135e+00 ; 6.330385e-01 ];
Tc_9  = [ -3.220745e+00 ; -1.658670e+02 ; 5.126650e+02 ];
omc_error_9 = [ 4.314048e-03 ; 4.646727e-03 ; 5.957287e-03 ];
Tc_error_9  = [ 2.973666e+00 ; 2.358655e+00 ; 1.792033e+00 ];

%-- Image #10:
omc_10 = [ 1.843958e+00 ; 1.365009e+00 ; 6.709406e-01 ];
Tc_10  = [ -1.146483e+02 ; -8.010816e+01 ; 4.556983e+02 ];
omc_error_10 = [ 5.039495e-03 ; 3.975421e-03 ; 6.593978e-03 ];
Tc_error_10  = [ 2.669853e+00 ; 2.127779e+00 ; 2.200389e+00 ];

