clc;
close all;
%clear;
%warning off;


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DEFINE PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 160408 CALIBRATION
%load 'D:\\ThuyNguyen\\project\\03-stereo, Basler camera\\data\\05-indoor,32canon\\160408 calibration(prepare to go to field)\\calib-MATLAB\\B15-B16.mat';

% 160408 CALIBRATION
%load 'D:\\ThuyNguyen\\project\\03-stereo, Basler camera\\data\\06-outdoor,32canon\\160613 CALIBRATION\\calib-MATLAB\\D25-D26.mat';
%load 'D:\\ThuyNguyen\\project\\03-stereo, Basler camera\\data\\06-outdoor,32canon\\160613 CALIBRATION\\calib-MATLAB\\D27-D28.mat';

% 160728 CALIBRATION
%load 'F:\\ThuyNguyen\\project\\03-stereo, Basler camera\\data\\06-outdoor,32canon\\160728 CALIBRATION-arm by arm\\calib-MATLAB\\A07-A08.mat'; % load GOOD calibration files
%load 'D:\\ThuyNguyen\\project\\03-stereo, Basler camera\\data\\06-outdoor,32canon\\160728 CALIBRATION-arm by arm\\calib-MATLAB\\B09-B10.mat';
%load 'F:\\ThuyNguyen\\project\\03-stereo, Basler camera\\data\\06-outdoor,32canon\\160728 CALIBRATION-arm by arm\\calib-MATLAB\\B11-B12.mat';
%load 'F:\\ThuyNguyen\\project\\03-stereo, Basler camera\\data\\06-outdoor,32canon\\160728 CALIBRATION-arm by arm\\calib-MATLAB\\C17-C18.mat';
%load 'F:\\ThuyNguyen\\project\\03-stereo, Basler camera\\data\\06-outdoor,32canon\\160728 CALIBRATION-arm by arm\\calib-MATLAB\\C19-C20.mat';
%load 'F:\\ThuyNguyen\\project\\03-stereo, Basler camera\\data\\06-outdoor,32canon\\160728 CALIBRATION-arm by arm\\calib-MATLAB\\C21-C22.mat';
%load 'F:\\ThuyNguyen\\project\\03-stereo, Basler camera\\data\\06-outdoor,32canon\\160728 CALIBRATION-arm by arm\\calib-MATLAB\\C23-C24.mat';
%load 'F:\\ThuyNguyen\\project\\03-stereo, Basler camera\\data\\06-outdoor,32canon\\160728 CALIBRATION-arm by arm\\calib-MATLAB\\D31-D32.mat';


% ===== 160528 Prof St.Clair tomato =====
% IMG_PATH_TEMPLATE = 'F:\\ThuyNguyen\\project\\03-stereo, Basler camera\\data\\06-outdoor,32canon\\160528 Prof St.Clair tomato\\OURS\\images\\%s_img missing\\%04d.jpg';
% 
% %IMG_PATH_LEFT     = 'A01';        IMG_PATH_RIGHT = 'A02';
% %IMG_PATH_LEFT     = 'A03';        IMG_PATH_RIGHT = 'A04';
% %IMG_PATH_LEFT     = 'A05';        IMG_PATH_RIGHT = 'A06';
% IMG_PATH_LEFT     = 'A07';        IMG_PATH_RIGHT = 'A08';
% 
% %IMG_PATH_LEFT     = 'B09';        IMG_PATH_RIGHT = 'B10';
% %IMG_PATH_LEFT     = 'B11';        IMG_PATH_RIGHT = 'B12';
% 
% IMG_FROM          = 15;
% IMG_TO            = 15;
% 
% N_DISPARITIES                   = 512;
% MATCHING_WIN_SIZE               = 15;       % width of square block
% MATCHING_CONTRAST_THR           = 0.5;      % default: 0.5
% MATCHING_UNIQUENESS_THR         = 45;       % default: 15 (if not used Denoise, it should be 65)
% MATCHING_DIST_THR               = [];       % default: [] (disable)
% 
% POINT_CLOUD_ROI                 = [-50, 50; -50, 50; 10, 50];
% 
% POINT_CLOUD_DENOISE_N_NEIGHBORS = 100;
% POINT_CLOUD_DENOISE_THR         = 0.1;


% ===== 160602 sunflowers(22),no lights =====
IMG_PATH_TEMPLATE = 'D:\\ThuyNguyen\\project\\03-stereo, Basler camera\\code\\160823 All-in-one\\build\\Release\\170301 test new lens 16mm\\matlab\\%s\\%04d.jpg';
%IMG_PATH_TEMPLATE = 'D:\\ThuyNguyen\\project\\03-stereo, Basler camera\\data\\06-outdoor,32canon\\160602 sunflowers(22),no lights\\OURS\\images\\%s\\%04d.jpg';
%IMG_PATH_LEFT     = 'B09';      IMG_PATH_RIGHT = 'B10';
%IMG_PATH_LEFT     = 'B11';      IMG_PATH_RIGHT = 'B12';
%IMG_PATH_LEFT     = 'B15';      IMG_PATH_RIGHT = 'B16';
%IMG_PATH_LEFT     = 'C17';      IMG_PATH_RIGHT = 'C18';
%IMG_PATH_LEFT     = 'C19';      IMG_PATH_RIGHT = 'C20';
%IMG_PATH_LEFT     = 'C21';      IMG_PATH_RIGHT = 'C22';
%IMG_PATH_LEFT     = 'C23';      IMG_PATH_RIGHT = 'C24';
%IMG_PATH_LEFT     = 'D25';      IMG_PATH_RIGHT = 'D26';
%IMG_PATH_LEFT     = 'D27';      IMG_PATH_RIGHT = 'D28';
IMG_PATH_LEFT     = 'D31';      IMG_PATH_RIGHT = 'D32';

IMG_FROM          = 1;
IMG_TO            = 1;

N_DISPARITIES                   = 512;
MATCHING_WIN_SIZE               = 17;       % width of square block
MATCHING_CONTRAST_THR           = 0.5;      % default: 0.5
MATCHING_UNIQUENESS_THR         = 45;       % default: 15 (if not used Denoise, it should be 65)
MATCHING_DIST_THR               = [];       % default: [] (disable)

POINT_CLOUD_ROI                 = [-50, 50; -50, 50; 10, 50];

POINT_CLOUD_DENOISE_N_NEIGHBORS = 100;
POINT_CLOUD_DENOISE_THR         = 0.1;


% ===== display =====
SHOW_ROTATED_IMG                = false;
SHOW_STEREO_ANAGLYPH            = false;
SHOW_COLOR_DISPARITY_MAP        = false;
SHOW_POINT_CLOUD                = false;
SAVE_POINT_CLOUD                = true;


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i = IMG_FROM : IMG_TO
    
    % --- load left and right images
    fprintf('Load    %s(%02d) and %s(%02d) images ... ', IMG_PATH_LEFT, i, IMG_PATH_RIGHT, i);
    timeStart = cputime;
    leftImg = imread(sprintf(IMG_PATH_TEMPLATE, IMG_PATH_LEFT, i));     % <---
    rightImg = imread(sprintf(IMG_PATH_TEMPLATE, IMG_PATH_RIGHT, i));   % <---
    timeElapsed = cputime - timeStart;
    fprintf('%.03f s \n', timeElapsed);
    
    
    % --- rotate the left image 90-degree LEFT, the right image 90-degree RIGHT (according to UC Davis system design)
    fprintf('Rotate  %s(%02d) and %s(%02d) images ... ', IMG_PATH_LEFT, i, IMG_PATH_RIGHT, i);
    timeStart = cputime;
    leftImgRotated = imrotate(leftImg, 90);     % <---
    rightImgRotated = imrotate(rightImg, -90);  % <---
    timeElapsed = cputime - timeStart;
    fprintf('%.03f s \n', timeElapsed);    
    
    if (SHOW_ROTATED_IMG) % show rotated images
        figure('Name', 'Rotated left');      imshow(leftImgRotated);     title('Rotated left image');
        figure('Name', 'Rotated right');     imshow(rightImgRotated);    title('Rotated right image');
    end
    
    
    % --- rectify left and right images based on calibration 
    fprintf('Rectify %s(%02d) and %s(%02d) images ... ', IMG_PATH_LEFT, i, IMG_PATH_RIGHT, i);
    timeStart = cputime;
    [leftImgRectified, rightImgRectified] = rectifyStereoImages(leftImgRotated, rightImgRotated, stereoParams); % <---
                                                                                               % stereoParams is a variable obtained from loading calib
    timeElapsed = cputime - timeStart;
    fprintf('%.03f s \n', timeElapsed);    
    
    if (SHOW_STEREO_ANAGLYPH) % show overlap (red and green) stereo image
        figure('Name', 'Stereo anaglyph');     imshow(stereoAnaglyph(leftImgRectified, rightImgRectified));    title('Stereo anaglyph image');
    end
    
    
    % --- stereo matching
    fprintf('\n');
    fprintf('SemiGlobal stereo matching (nDisparities = %d, winSize = %d, contrastThr = %f, uniquenessThr = %d, distThr = %d) ... ', ...
             N_DISPARITIES, MATCHING_WIN_SIZE, MATCHING_CONTRAST_THR, MATCHING_UNIQUENESS_THR, MATCHING_DIST_THR);
    timeStart = cputime;
    disparityMap = disparity(rgb2gray(leftImgRectified), rgb2gray(rightImgRectified), ...   % <---
                             'Method',              'SemiGlobal',           ...
                             'DisparityRange',      [0 N_DISPARITIES],      ...
                             'BlockSize',           MATCHING_WIN_SIZE,      ...
                             'ContrastThreshold',   MATCHING_CONTRAST_THR,  ... % 0 < thr <= 1. Increase -> fewer pixels being marked as unreliable
                             'UniquenessThreshold', MATCHING_UNIQUENESS_THR,... % (0 to disable) non-neg integer defining min value of uniqueness. If a pixel is less unique,
                                                                            ... %      the disparity computed for it is less reliable. Increase-> mark more pixels unreliable
                             'DistanceThreshold',   MATCHING_DIST_THR);         % ([] to disable) non-neg integer defining max dist for left-right check. Increase-> fewer
                                                                                %      pixels being marked as unreliable  
    timeElapsed = cputime - timeStart;
    fprintf('%.03f s \n', timeElapsed);          
    
    if (SHOW_COLOR_DISPARITY_MAP) % show matching result
        figure('Name', 'Disparity');     imshow(disparityMap, [0, N_DISPARITIES]);   title('Disparity map');
        colormap jet
        colorbar
    end
    
    
    % --- reconstruct from the disparity map to a 3D point cloud, based on the (calibrated) perspective transform matrix Q
    fprintf('\n');
    fprintf('Generate point cloud based on disparity map ... ');
    timeStart = cputime;
    xyzPoints = reconstructScene(disparityMap, stereoParams);           % <---
    pointCloud3D = pointCloud(xyzPoints, 'Color', leftImgRectified);    % <--- convert points3D structure to pointCloud structure (with color) to process or save to file
    timeElapsed = cputime - timeStart;
    fprintf('%.03f s \n', timeElapsed);
    
    fprintf('   Point count = %d \n', pointCloud3D.Count);
    fprintf('   X range     = [%f, %f] \n', pointCloud3D.XLimits(1), pointCloud3D.XLimits(2));
    fprintf('   Y range     = [%f, %f] \n', pointCloud3D.YLimits(1), pointCloud3D.YLimits(2));
    fprintf('   Z range     = [%f, %f] \n', pointCloud3D.ZLimits(1), pointCloud3D.ZLimits(2));
    
    
    % --- filter out invalid points in the point cloud
    fprintf('\n');
    fprintf('Remove invalid points ... ');
    timeStart = cputime;
    [pointCloud3Dvalid, ~]= removeInvalidPoints(pointCloud3D);  % <---
    timeElapsed = cputime - timeStart;
    fprintf('%.03f s \n', timeElapsed);
    
    fprintf('   Point count = %d \n', pointCloud3Dvalid.Count);
    fprintf('   X range     = [%f, %f] \n', pointCloud3Dvalid.XLimits(1), pointCloud3Dvalid.XLimits(2));
    fprintf('   Y range     = [%f, %f] \n', pointCloud3Dvalid.YLimits(1), pointCloud3Dvalid.YLimits(2));
    fprintf('   Z range     = [%f, %f] \n', pointCloud3Dvalid.ZLimits(1), pointCloud3Dvalid.ZLimits(2));
    
    
    % --- find points in ROI   
    fprintf('\n');
    fprintf('Find points in ROI ... '); 
    timeStart = cputime;
    pointCloud3DroiIndices = findPointsInROI(pointCloud3Dvalid, POINT_CLOUD_ROI);
    pointCloud3Droi = select(pointCloud3Dvalid, pointCloud3DroiIndices);
    timeElapsed = cputime - timeStart;
    fprintf('%.03f s \n', timeElapsed);
    
    fprintf('   Point count = %d \n', pointCloud3Droi.Count);
    fprintf('   X range     = [%f, %f] \n', pointCloud3Droi.XLimits(1), pointCloud3Droi.XLimits(2));
    fprintf('   Y range     = [%f, %f] \n', pointCloud3Droi.YLimits(1), pointCloud3Droi.YLimits(2));
    fprintf('   Z range     = [%f, %f] \n', pointCloud3Droi.ZLimits(1), pointCloud3Droi.ZLimits(2));
    
    
%     % --- denoise the point cloud
%     fprintf('\n');
%     fprintf('Denoise the point cloud ... '); 
%     timeStart = cputime;
%     pointCloud3Ddenoised = pcdenoise(pointCloud3Droi, ...
%                                      'NumNeighbors', POINT_CLOUD_DENOISE_N_NEIGHBORS, ... % estimate mean of avg dist to neighbors of all points. 
%                                                                                       ... % Decrease-> more sensitive to noise, less computation
%                                      'Threshold',    POINT_CLOUD_DENOISE_THR);            % By default, the thr is 1 std deviation from mean of avg dist to neighbors of 
%                                                                                           % all points. A point considered an outlier if avg dist to its k-nearest 
%                                                                                           % neighbors is above the specified thr
%     timeElapsed = cputime - timeStart;
%     fprintf('%.03f s \n', timeElapsed);
%     
%     fprintf('   Point count = %d \n', pointCloud3Ddenoised.Count);
%     fprintf('   X range     = [%f, %f] \n', pointCloud3Ddenoised.XLimits(1), pointCloud3Ddenoised.XLimits(2));
%     fprintf('   Y range     = [%f, %f] \n', pointCloud3Ddenoised.YLimits(1), pointCloud3Ddenoised.YLimits(2));
%     fprintf('   Z range     = [%f, %f] \n', pointCloud3Ddenoised.ZLimits(1), pointCloud3Ddenoised.ZLimits(2));
    
    
    % --- show point cloud
    if (SHOW_POINT_CLOUD)
        figure('Name', 'Point cloud');  
        pcshow(pointCloud3Droi);
        %pcshow(pointCloud3Ddenoised);        
        title('Point cloud')
        xlabel('X (inch)')
        ylabel('Y (inch)')
        zlabel('Z (inch)')
        drawnow
    end
    
    
    % --- save the point cloud to PLY file 
    if (SAVE_POINT_CLOUD)
        fprintf('\n');
        fprintf('Save point cloud to PLY file ... ');
        timeStart = cputime;    
        pcwrite(pointCloud3Droi, [IMG_PATH_LEFT '-' IMG_PATH_RIGHT '_' num2str(i)], 'PLYFormat', 'binary'); 
        %pcwrite(pointCloud3Ddenoised, [IMG_PATH_LEFT '-' IMG_PATH_RIGHT '_' num2str(i)], 'PLYFormat', 'binary');         
        timeElapsed = cputime - timeStart;
        fprintf('%.03f s \n', timeElapsed);
    end
    
end

fprintf('\n');
disp('DONE!');


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% REFERENCES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% [1] Depth Estimation From Stereo Video: http://www.mathworks.com/help/vision/examples/depth-estimation-from-stereo-video.html
%     --> functions: rectifyStereoImages, disparity (SemiGlobal), reconstructScene, pointCloud
%
% [2] OpenCV Camera Calibration and 3D Reconstruction: http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
%     --> Perspective transformation matrix Q = A * [R|T] where Q is 3x4, A is camera matrix 3x3 [fx  0 cx   and [R|T] is 3x4 of rotation and translation matrices
%                                                                                                  0 fy cy
%                                                                                                  0  0  1]
%
% [3] Camera calibration With OpenCV: http://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
%     --> Matlab's Stereo Calibrator provides calibration result as radial and tangential distortion parameters are in 2 separate arrays (3-element and 2-element arrays).
%         But in OpenCV's format, distortion paramters are in 1 array [k1 k2 p1 p2 k3] where k is radial-distort paramter, and p is tangential-distort parameter
%