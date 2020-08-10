% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 693.145135900556852 ; 690.655688360068098 ];

%-- Principal point:
cc = [ 285.684112386684717 ; 244.440334109729406 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.104630982984186 ; -0.244719852328428 ; -0.000017963044467 ; 0.000419189624379 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 1.956166921197036 ; 1.926426398624857 ];

%-- Principal point uncertainty:
cc_error = [ 3.379595325702257 ; 2.549153050242302 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.014314634461038 ; 0.075444135800384 ; 0.001532448777506 ; 0.002150343583148 ; 0.000000000000000 ];

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
omc_1 = [ 2.077095e+00 ; 2.167436e+00 ; -1.528531e-01 ];
Tc_1  = [ -7.153115e+01 ; -8.811284e+01 ; 3.905683e+02 ];
omc_error_1 = [ 3.122097e-03 ; 3.594128e-03 ; 7.154986e-03 ];
Tc_error_1  = [ 1.921132e+00 ; 1.437769e+00 ; 1.246764e+00 ];

%-- Image #2:
omc_2 = [ 2.175354e+00 ; 2.247384e+00 ; 1.754978e-01 ];
Tc_2  = [ -8.893505e+01 ; -5.595507e+01 ; 4.373690e+02 ];
omc_error_2 = [ 4.078593e-03 ; 3.561343e-03 ; 8.442897e-03 ];
Tc_error_2  = [ 2.147844e+00 ; 1.631030e+00 ; 1.472827e+00 ];

%-- Image #3:
omc_3 = [ -1.954824e+00 ; -2.134565e+00 ; 4.085088e-01 ];
Tc_3  = [ -6.403249e+01 ; -8.810250e+01 ; 4.338437e+02 ];
omc_error_3 = [ 3.557482e-03 ; 3.392533e-03 ; 6.563563e-03 ];
Tc_error_3  = [ 2.123390e+00 ; 1.592224e+00 ; 1.282315e+00 ];

%-- Image #4:
omc_4 = [ 1.854766e+00 ; 1.801717e+00 ; -6.182370e-01 ];
Tc_4  = [ -9.281188e+01 ; -4.414013e+01 ; 4.758596e+02 ];
omc_error_4 = [ 2.658830e-03 ; 3.849557e-03 ; 5.980611e-03 ];
Tc_error_4  = [ 2.318599e+00 ; 1.755113e+00 ; 1.369883e+00 ];

%-- Image #5:
omc_5 = [ 1.981026e+00 ; 1.701463e+00 ; 2.168297e-01 ];
Tc_5  = [ -5.529020e+01 ; -8.101685e+01 ; 4.260970e+02 ];
omc_error_5 = [ 3.665987e-03 ; 3.382225e-03 ; 6.076273e-03 ];
Tc_error_5  = [ 2.095619e+00 ; 1.565076e+00 ; 1.401008e+00 ];

%-- Image #6:
omc_6 = [ -1.842135e+00 ; -1.905758e+00 ; -4.672417e-01 ];
Tc_6  = [ -7.020095e+01 ; -8.573108e+01 ; 3.660924e+02 ];
omc_error_6 = [ 2.419922e-03 ; 3.894158e-03 ; 6.157651e-03 ];
Tc_error_6  = [ 1.812088e+00 ; 1.359172e+00 ; 1.274154e+00 ];

%-- Image #7:
omc_7 = [ 1.630991e+00 ; 1.511881e+00 ; -1.306418e-02 ];
Tc_7  = [ -1.077335e+02 ; -6.643479e+01 ; 4.691577e+02 ];
omc_error_7 = [ 3.162617e-03 ; 3.895183e-03 ; 5.005681e-03 ];
Tc_error_7  = [ 2.301786e+00 ; 1.740803e+00 ; 1.561299e+00 ];

%-- Image #8:
omc_8 = [ 1.778480e+00 ; 2.188553e+00 ; -1.194707e+00 ];
Tc_8  = [ -7.412797e+01 ; -8.996109e+01 ; 5.188018e+02 ];
omc_error_8 = [ 2.151537e-03 ; 4.608575e-03 ; 6.822106e-03 ];
Tc_error_8  = [ 2.551858e+00 ; 1.925409e+00 ; 1.352610e+00 ];

%-- Image #9:
omc_9 = [ -1.338237e+00 ; -1.817660e+00 ; 6.342682e-01 ];
Tc_9  = [ 1.170764e+01 ; -1.299086e+02 ; 4.972625e+02 ];
omc_error_9 = [ 3.477211e-03 ; 3.803356e-03 ; 4.847153e-03 ];
Tc_error_9  = [ 2.454146e+00 ; 1.831032e+00 ; 1.354868e+00 ];

%-- Image #10:
omc_10 = [ 1.849316e+00 ; 1.368410e+00 ; 6.711382e-01 ];
Tc_10  = [ -8.632879e+01 ; -5.858285e+01 ; 4.795387e+02 ];
omc_error_10 = [ 4.032134e-03 ; 3.317776e-03 ; 5.304497e-03 ];
Tc_error_10  = [ 2.366789e+00 ; 1.776862e+00 ; 1.706234e+00 ];

