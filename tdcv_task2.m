%to do: plot 3d sift points, feature matching, 3d-2d projection -> thats it.
setup;

load('training_image1');
load('training_image2');
load('training_image3');
load('training_image4');
load('training_image5');
load('training_image6');
load('training_image7');
load('training_image8');

M_i = [0 0.063 0.093;
       0.165 0.063 0.093;
       0.165 0 0.093;
       0 0 0.093;
       0 0.063 0;
       0.165 0.063 0;
       0.165 0 0;
       0 0 0];
   
 figure(2);
 pcshow(M_i,[1 0 1],'Marker',15);
 hold on;
% plot3(sift_location_image1(:,1),sift_location_image1(:,2),sift_location_image1(:,3),'r+');
% plot3(sift_location_image2(:,1),sift_location_image2(:,2),sift_location_image2(:,3),'r+');
% plot3(sift_location_image3(:,1),sift_location_image3(:,2),sift_location_image3(:,3),'r+');
% plot3(sift_location_image4(:,1),sift_location_image4(:,2),sift_location_image4(:,3),'r+');
% plot3(sift_location_image5(:,1),sift_location_image5(:,2),sift_location_image5(:,3),'r+');
% plot3(sift_location_image6(:,1),sift_location_image6(:,2),sift_location_image6(:,3),'r+');
% plot3(sift_location_image7(:,1),sift_location_image7(:,2),sift_location_image7(:,3),'r+');
% plot3(sift_location_image8(:,1),sift_location_image8(:,2),sift_location_image8(:,3),'r+');

%Making full_sift_world_point_matrix and its descriptor matrix

full_sift_matrix = [sift_location_image1;sift_location_image2;sift_location_image3;sift_location_image4;sift_location_image5;sift_location_image6;sift_location_image7;sift_location_image8];
           
plot3(full_sift_matrix(:,1),full_sift_matrix(:,2),full_sift_matrix(:,3),'b+');

full_descriptor_matrix = [descriptor_matrix_image1 descriptor_matrix_image2 descriptor_matrix_image3 descriptor_matrix_image4 descriptor_matrix_image5 descriptor_matrix_image6 descriptor_matrix_image7 descriptor_matrix_image8];  
valid_pt = 0;

TF = isnan(full_sift_matrix(:,1));
TF = ~TF;
full_sift_matrix_new = full_sift_matrix(TF,:);
full_descriptor_matrix_new = full_descriptor_matrix(:,TF);

% for i=1:numel(full_sift_matrix(:,1))
%     
%     if isnan(full_sift_matrix(i,1)) == 1
%         
%         valid_pt = valid_pt + 1;
% 
%         full_sift_matrix_new(valid_pt,:) = full_sift_matrix(i,:);
%         full_descriptor_matrix(:,valid_pt) = full_descriptor_matrix(:,i);        
%         
%     end;
%     
% end;  

I = imread('images/detection/DSC_9754.JPG');
%original_I = I;
I = rgb2gray(I);
camera_m = cameraIntrinsics([2960.37845,2960.37845],[1841.68855,1235.23369],size(I));
figure(1), imshow(I);
hold on;
intrinsic_matrix = [2960.37845 0 1841.68855;
                    0 2960.37845 1235.23369;
                    0 0 1];
                
[frames_sift, descrs_sift] = getFeatures(I, 'peakThreshold', 0.01) ;

x_sift = frames_sift(1,:);
y_sift = frames_sift(2,:);
number_sift_pts = numel(x_sift);

full_descriptor_matrix_new = single(full_descriptor_matrix_new);

%testing_desc = full_descriptor_matrix(:,1:1112);

% Find for each descriptor in im1 the closest descriptor in im2
%nn = findNeighbours(descrs_sift,full_descriptor_matrix);

[matches, scores] = vl_ubcmatch(descrs_sift,full_descriptor_matrix_new);

% Construct a matrix of matches. Each column stores two index of
% matching features in im1 and im2
%matches = [1:size(descrs_sift,2) ; nn(1,:)];

plot(x_sift,y_sift,'b+');

figure(2);
plot3(full_sift_matrix(:,1),full_sift_matrix(:,2),full_sift_matrix(:,3),'b+');

plot3(full_sift_matrix(matches(2,:),1),full_sift_matrix(matches(2,:),2),full_sift_matrix(matches(2,:),3),'ro');

figure(1);
plot(x_sift(matches(1,:)),y_sift(matches(1,:)),'ro');

input2dMat = zeros(numel(matches(1,:)),2);
%input3dMat = zeros(numel(matches(1,:)),3);

for i=1:numel(matches(1,:))
match_no = matches(1,i);
input2dMat(i,:) = [x_sift(match_no) y_sift(match_no)];
%input3dMat(i,: 
end
%input2dMat(:,1) = x_sift(match_no(1,:));
%input2dMat(:,2) = y_sift(match_no(1,:));
input3dMat = full_sift_matrix_new(matches(2,:),:);
%imageI = I;
% for i=1:numel(matches(1,:))
%     
% figure(2);
% plot3(full_sift_matrix(matches(2,i),1),full_sift_matrix(matches(2,i),2),full_sift_matrix(matches(2,i),3),'ro');
% figure(1);
% plot(x_sift(matches(1,i)),y_sift(matches(1,i)),'ro');
% 
% end;
    
%% RANSAC
% input2dMat, input3dMat
% T := #trials
% s := #points in one sample set
% we need at least 4 vertices to accomplish P4P-Algorithm
% 
s = 4; 
% e := outlier ratio
e = 0.4;
% p := propability
p = 0.99;
% 1-p := probability, >= 1 sample set is outlier-free
% th := threshold e.g. 30 pixels
threshold = 1;
% e = #outliers / #data_points
% 1-p = (1-(1-e)^s)^T    fail T times
pWithoutOutliers = 1-(1-e)^s;
% ==> set p to solve T
% T = log(1-p) / log(1-(1-e)^s)
T = log(1-p) / log(pWithoutOutliers);

%% #data_points = e.g. 100 (2D-and-3D-correspondences-match)
numDataPt = size(input2dMat,1); %#correspondences

% BM := best model having most bestInliers => we do not calculate this in
% the function. Instead, we do it in the loop of detection-images
% bestInliers := array of indices of inliers
% imageI := imageindex, to which image this algorithm is implemented for
% output2dMat, output3dMat

%% loop
currentLoop = 1;
max_s = 0;
% set up sample list which stores indices of inliers:
best_sample = 0;
%T = 2;
while currentLoop <= T %change

%     randIndex=single.empty;
%     index_vector=single.empty;
%     selected2dM=single.empty;
%     selected3dM=single.empty;
%     Image_Pts=single.empty;
%     World_Pts=single.empty;
    
    % Randomly select 4 indices
    if numDataPt < 4
        disp('Indices not enough!');
    else
        randIndex = randsample(numDataPt,4);
        index_vector = [1:numDataPt];
    end
    
    %set the correspondences as inliers (using selected indices)
    selected2dM = input2dMat(randIndex,:);
    selected3dM = input3dMat(randIndex,:);
    
    %% estimate world position function ==> to get R|T
    Image_Pts = selected2dM;
    World_Pts = selected3dM;
    Image_Pts = vertcat(Image_Pts, Image_Pts);
    World_Pts = vertcat(World_Pts, World_Pts);

    %Rotation and Translation matrices in world coordination
    [worldR,worldT] = estimateWorldCameraPose(Image_Pts,World_Pts,camera_m);
    %[worldR,worldT] = estimateWorldCameraPose(Image_Pts,World_Pts,camera_m,'MaxReprojectionError',2);
    %[worldR,worldT] = estimateWorldCameraPose(input2dMat,input3dMat,camera_m);
%currentLoop = currentLoop + 1;
%end
    %Find elements in one array not in another
    uncommon_index=setdiff(index_vector,randIndex);
    
    left2dM = input2dMat(uncommon_index,:);
    left3dM = input3dMat(uncommon_index,:);
    
    %Location of projected points
    
    [extrinsic_rotation,extrinsic_translation] = cameraPoseToExtrinsics(worldR,worldT);

    p1transpose=transpose(extrinsic_rotation);
    p134=[p1transpose(:,1:3) transpose(extrinsic_translation)];
    projected2dM=single.empty;

    for i=1:numel(left3dM(:,1))  
    reproj_pt = intrinsic_matrix*p134*transpose([left3dM(i,:) 1]);
    projected2dM = [projected2dM;[reproj_pt(1)/reproj_pt(3),reproj_pt(2)/reproj_pt(3)]];
    end;
    
    % distance between matrix of projected point and given 2d-Matrix
    x1 = projected2dM(:,1);
    y1 = projected2dM(:,2);
    x2 = left2dM(:,1);
    y2 = left2dM(:,2);
    dist = sqrt((x2-x1).^2 + (y2-y1).^2);
    
    index_inlier = find(dist < threshold);
    
    s_ii = numel(index_inlier); %current size of index_inlier

    if s_ii > max_s
        max_s = s_ii;
        best_sample = index_inlier;
        best_left2dM = left2dM;
        best_left3dM = left3dM;
        best_R = worldR;
        best_T = worldT; 
        %BM = imageI;
    end    
    currentLoop = currentLoop + 1;    
end

for i=1:numel(best_sample)
    Image_Pts_new(i,:) = best_left2dM(best_sample(i),:);
    World_Pts_new(i,:) = best_left3dM(best_sample(i),:);    
end; 
    
[worldR_new,worldT_new] = estimateWorldCameraPose(Image_Pts_new,World_Pts_new,camera_m);
    
[extrinsic_rotation_new,extrinsic_translation_new] = cameraPoseToExtrinsics(worldR_new,worldT_new);

p1transpose_new=transpose(extrinsic_rotation_new);
p134_new=[p1transpose_new(:,1:3) transpose(extrinsic_translation_new)];
projected2dM_new=single.empty;

for i=1:8  
reproj_pt = intrinsic_matrix*p134_new*transpose([M_i(i,:) 1]);
projected2dM_new = [projected2dM_new;[reproj_pt(1)/reproj_pt(3),reproj_pt(2)/reproj_pt(3)]];
end;

plot(projected2dM_new(:,1),projected2dM_new(:,2),'b+'); 

%total_faces = [1432 5876 1584 2376 4873 1562];

xv = [projected2dM_new(1,1) projected2dM_new(4,1) projected2dM_new(3,1) projected2dM_new(2,1) projected2dM_new(1,1)];
yv = [projected2dM_new(1,2) projected2dM_new(4,2) projected2dM_new(3,2) projected2dM_new(2,2) projected2dM_new(1,2)];

plot(xv,yv,'LineWidth',2) % polygon

xv = [projected2dM_new(5,1) projected2dM_new(8,1) projected2dM_new(7,1) projected2dM_new(6,1) projected2dM_new(5,1)];
yv = [projected2dM_new(5,2) projected2dM_new(8,2) projected2dM_new(7,2) projected2dM_new(6,2) projected2dM_new(5,2)];

plot(xv,yv,'LineWidth',2) % polygon

xv = [projected2dM_new(1,1) projected2dM_new(5,1) projected2dM_new(8,1) projected2dM_new(4,1) projected2dM_new(1,1)];
yv = [projected2dM_new(1,2) projected2dM_new(5,2) projected2dM_new(8,2) projected2dM_new(4,2) projected2dM_new(1,2)];

plot(xv,yv,'LineWidth',2) % polygon

xv = [projected2dM_new(2,1) projected2dM_new(3,1) projected2dM_new(7,1) projected2dM_new(6,1) projected2dM_new(2,1)];
yv = [projected2dM_new(2,2) projected2dM_new(3,2) projected2dM_new(7,2) projected2dM_new(6,2) projected2dM_new(2,2)];

plot(xv,yv,'LineWidth',2) % polygon

xv = [projected2dM_new(4,1) projected2dM_new(8,1) projected2dM_new(7,1) projected2dM_new(3,1) projected2dM_new(4,1)];
yv = [projected2dM_new(4,2) projected2dM_new(8,2) projected2dM_new(7,2) projected2dM_new(3,2) projected2dM_new(4,2)];

plot(xv,yv,'LineWidth',2) % polygon

xv = [projected2dM_new(1,1) projected2dM_new(5,1) projected2dM_new(6,1) projected2dM_new(2,1) projected2dM_new(1,1)];
yv = [projected2dM_new(1,2) projected2dM_new(5,2) projected2dM_new(6,2) projected2dM_new(2,2) projected2dM_new(1,2)];

plot(xv,yv,'LineWidth',2) % polygon

    