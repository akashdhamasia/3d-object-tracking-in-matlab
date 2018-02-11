%to do: 
%ask for total no. of vertices, store edges and faces -> Also take the face vertices
%increase the no. of points(middle point of vertices) to be given to estimateWorldCameraPose function 
%-> after that you get the rotation and translation matrix
%sift features and descriptors -> storage and backprojection: equation of a plane and frame the equation for backprojection
%Plot the sift points on 3D model.
%Do it for all the training images and save world_coordinates with their descriptors
%Test image -> sift features -> do the matching -> find the worldcoordinates -> find the Rotation and Translation matrix. 
%plot the 3d model on 2d test images.

%All the tasks

%to_do: 1. apply ray plane intersection on sift features -> find the
%intersection -> if its correct -> do it for all the faces of one training image -> 
%store sift features descriptor, its 2d image location, its 3d image location, its face_no and its image_no. 
%Similarily do it for all the training images -> only store unique sift features.    

%task2: find sift features of whole test image -> find the correspondense
%between test image and 3d object using sift features matching -> select random 4 correspondences -> 
%find R and T using worldpositionestimate function -> project all the
%remaining correspondenses using R and T on the 2d image and calculate
%displacement between the actual matching and projected point -> if it is
%smaller than some threshold -> store 

%task3 -> With the R and T which we get from task2 -> optimise R and
%T using levenberg marquardt algorithm -> tasks are: to make a iterative
%loop to update R and T and calculate jacobian.

%task4 -> just for testing.

%%%%%%%%%%%%%%%%%%%%%

setup;
%any;
%any2;
%any3;

%ptCloud = pcread('model/teabox.ply');
%figure, pcshow(ptCloud);

%% coordinates of 3d-object
%M_i := global_obj_co = 6:3;
M_i = [0 0.063 0.093;
       0.165 0.063 0.093;
       0.165 0 0.093;
       0 0 0.093;
       0 0.063 0;
       0.165 0.063 0;
       0.165 0 0;
       0 0 0];
   
%store faces in anti-clockwise direction	   
total_faces = [1432 5876 1584 2376 4873 1562]; %just segment out the vertices and compare with the user input vertices to get the faceid.
total_edges = [12 23 34 14 56 67 78 58 26 37 15 48]; %same procedure
diagonal_per_face = [24 36 68 45 38 16]; %same procedure
total_triangle_faces = [142 243 586 687 154 458 236 637 483 387 152 256];

I = imread('images/init_texture/DSC_9743.JPG');
%original_I = I;
I = rgb2gray(I);
camera_m = cameraIntrinsics([2960.37845,2960.37845],[1841.68855,1235.23369],size(I));
figure(1), imshow(I);
hold on;
no_vertices = input('How many vertices are visible? [6 or 7] ');
while (no_vertices ~= 6) && (no_vertices ~= 7)
    no_vertices = input('Wrong input. How many vertices are visible? [6 or 7] ');
end
%no_vertices = 6;
%8 images(nt now), input number of vertices, (x,y,label)
%train_i = zeros(no_vertices,4,8);
train_i = zeros(no_vertices,3);
imagePoints_1 = zeros(25,2);
worldPoints_1 = zeros(25,3);

for i = 1:no_vertices
sp=getrect;
% Get the x and y co-ordinates
sp(3)= sp(1) + sp(3); %xmax
sp(4)= sp(2) + sp(4); %ymax
% Index into the original image to create the new image
MM=I(sp(2):sp(4), sp(1):sp(3));
%% Find the corners.
corners = detectHarrisFeatures(MM); hold on;
pt = corners.selectStrongest(1);
x = pt.Location(1,1);
y = pt.Location(1,2);
disp(x);
disp(y);
ac_pt_x = sp(1) + x;
ac_pt_y = sp(2) + y;
plot(ac_pt_x,ac_pt_y,'r+', 'MarkerSize', 10);
train_i(i,1) = ac_pt_x;
train_i(i,2) = ac_pt_y;
prompt = 'Vertices no. ';
train_i(i,3) = input(prompt);

imagePoints_1(i,1) = ac_pt_x;
imagePoints_1(i,2) = ac_pt_y;
worldPoints_1(i,1) = M_i(train_i(i,3), 1);
worldPoints_1(i,2) = M_i(train_i(i,3), 2);
worldPoints_1(i,3) = M_i(train_i(i,3), 3);

end
disp('train_i');
disp(train_i(:,:));
counter = no_vertices;

%calculate the no. of edges and diagonal and faceid.

%edge
matr_e_midpt = zeros(12,2);

for i = 1:12 

    chr = int2str(total_edges(i));

	for j = 1:no_vertices
	
		if str2num(chr(1:1)) == train_i(j,3) 
			vertex_1 = j;%train_i(j,1:2); 			
        end
        
        if str2num(chr(2:2)) == train_i(j,3)
			vertex_2 = j;%train_i(j,1:2); 			
        end
					
	end
	
	if (vertex_1 > 0) && (vertex_2 > 0)
	   %plot the edges in the graph (line function)
       %l_range = linspace(vertex_1,vertex_2);
       %line(vertex_1,vertex_2);
       
	   %store the midpoints
       %midpt = (vertex_1 + vertex_2)/2;
       %matr_e_midpt(i,:) = midpt;
	   
	   counter = counter + 1;
	   
        %midpt_x_local = (train_i(vertex_1,1) + train_i(vertex_2,1))/2;
        %midpt_y_local = (train_i(vertex_1,2) + train_i(vertex_2,2))/2;
		
		imagePoints_1(counter,1) = (train_i(vertex_1,1) + train_i(vertex_2,1))/2;
		imagePoints_1(counter,2) = (train_i(vertex_1,2) + train_i(vertex_2,2))/2;
		worldPoints_1(counter,1) = (M_i(train_i(vertex_1,3), 1) + M_i(train_i(vertex_2,3), 1))/2;
		worldPoints_1(counter,2) = (M_i(train_i(vertex_1,3), 2) + M_i(train_i(vertex_2,3), 2))/2;
		worldPoints_1(counter,3) = (M_i(train_i(vertex_1,3), 3) + M_i(train_i(vertex_2,3), 3))/2;


    end 
    %reinitialise
	vertex_1 = 0;
	vertex_2 = 0;
end 

%diagonal
%matr_diag_midp = zeros(6,2);

for i = 1:6
    
    chr = int2str(diagonal_per_face(i));
    
    for j = 1:no_vertices
	
		if str2num(chr(1:1)) == train_i(j,3) 
			vertex_1 = j;%train_i(j,1:2); 			
        end
        
        if str2num(chr(2:2)) == train_i(j,3)
			vertex_2 = j;%train_i(j,1:2); 			
        end
					
	end
	
	if vertex_1 > 0 && vertex_2 > 0
	   %plot the edges in the graph (line function)
       %l_range = linspace(vertex_1,vertex_2);
       %line(vertex_1,vertex_2);
       
	   %store the points
		   
        %midpt_x = (train_i(vertex_1,1) + train_i(vertex_2,1))/2;
        %midpt_y = (train_i(vertex_1,2) + train_i(vertex_2,2))/2;
		counter = counter + 1;
		
		imagePoints_1(counter,1) = (train_i(vertex_1,1) + train_i(vertex_2,1))/2;
		imagePoints_1(counter,2) = (train_i(vertex_1,2) + train_i(vertex_2,2))/2;
		worldPoints_1(counter,1) = (M_i(train_i(vertex_1,3), 1) + M_i(train_i(vertex_2,3), 1))/2;
		worldPoints_1(counter,2) = (M_i(train_i(vertex_1,3), 2) + M_i(train_i(vertex_2,3), 2))/2;
		worldPoints_1(counter,3) = (M_i(train_i(vertex_1,3), 3) + M_i(train_i(vertex_2,3), 3))/2;

        %matr_diag_midp(i,:) = midpt;
    end 
    
	vertex_1 = 0;
	vertex_2 = 0;

end

imagePoints = imagePoints_1(1:counter,:);
worldPoints = worldPoints_1(1:counter,:);

%imagePoints = imagePoints_1(1:no_vertices,:);
%worldPoints = worldPoints_1(1:no_vertices,:);

%imagePoints = [imagePoints;
%               imagePoints];
           
%worldPoints = [worldPoints;
%                worldPoints];
            
% train_i = 1.0e+03 * [1.3465    1.1377    0.0040;
%     1.3752    1.0252    0.0010;
%     2.2348    1.0076    0.0020;
%     2.3104    1.1152    0.0030;
%     2.2803    1.5912    0.0070;
%     1.3739    1.6144    0.0080];
%     
% %train_i = any;
% %1.0e+03 *[1.3485    1.1397    0.0040;
% %     1.3759    1.0322    0.0010;
% %     2.2394    1.0104    0.0020;
% %     2.3124    1.1182    0.0030;
% %     2.2818    1.5926    0.0070;
% %     1.3753    1.6171    0.0080];
% 
% imagePoints = 1.0e+03 * [1.3465    1.1377;
%     1.3752    1.0252;
%     2.2348    1.0076;
%     2.3104    1.1152;
%     2.2803    1.5912;
%     1.3739    1.6144;
%     1.8050    1.0164;
%     2.2726    1.0614;
%     1.8285    1.1264;
%     1.3608    1.0815;
%     1.8271    1.6028;
%     2.2954    1.3532;
%     1.3602    1.3761;
%     1.7906    1.0727;
%     1.8422    1.3648];
% % 
% %imagePoints = any2; 
% % 1.0e+03*[1.3485    1.1397;
% %     1.3759    1.0322;
% %     2.2394    1.0104;
% %     2.3124    1.1182;
% %     2.2818    1.5926;
% %     1.3753    1.6171;
% %     1.8076    1.0213;
% %     2.2759    1.0643;
% %     1.8305    1.1289;
% %     1.3622    1.0860;
% %     1.8285    1.6049;
% %     2.2971    1.3554;
% %     1.3619    1.3784;
% %     1.7940    1.0751;
% %     1.8439    1.3676];
% 
% worldPoints = [0         0    0.0930;
%          0    0.0630    0.0930;
%     0.1650    0.0630    0.0930;
%     0.1650         0    0.0930;
%     0.1650         0         0;
%          0         0         0;
%     0.0825    0.0630    0.0930;
%     0.1650    0.0315    0.0930;
%     0.0825         0    0.0930;
%          0    0.0315    0.0930;
%     0.0825         0         0;
%     0.1650         0    0.0465;
%          0         0    0.0465;
%     0.0825    0.0315    0.0930;
%     0.0825         0    0.0465]; 

 %worldPoints = any3;
%[0         0    0.0930;
%          0    0.0630    0.0930;
%     0.1650    0.0630    0.0930;
%     0.1650         0    0.0930;
%     0.1650         0         0;
%          0         0         0;
%     0.0825    0.0630    0.0930;
%     0.1650    0.0315    0.0930;
%     0.0825         0    0.0930;
%          0    0.0315    0.0930;
%     0.0825         0         0;
%     0.1650         0    0.0465;
%          0         0    0.0465;
%     0.0825    0.0315    0.0930;
%     0.0825         0    0.0465];

% disp('imagePoints');
% disp(imagePoints);
% disp('worldPoints');
% disp(worldPoints);

[worldOrientation,worldLocation,inlierIdx] = estimateWorldCameraPose(imagePoints,worldPoints,camera_m);

p1=worldOrientation;
t1=worldLocation;
kinv=inv(transpose(camera_m.IntrinsicMatrix));
rotation_matrix = worldOrientation;
translation_matrix = transpose(worldLocation);
%disp('inlierIdx');
%disp(inlierIdx);
% disp('worldOrientation');
% disp(worldOrientation);
% disp('worldLocation');
% disp(worldLocation);

intrinsic_matrix = [2960.37845 0 1841.68855;
                    0 2960.37845 1235.23369;
                    0 0 1];
                
%testing 3d to 2d projection using R and T 

% testin = zeros(3,1);
% [test_rotation,test_translation] = cameraPoseToExtrinsics(worldOrientation,worldLocation);
% 
% p1transpose=transpose(test_rotation);
% p134=[p1transpose(:,1:3) transpose(test_translation)];
% reproj_cm_pt=single.empty;
% reproj_pt=intrinsic_matrix*p134*transpose([M_i(1,:) 1]);
% reproj_cm_pt= [reproj_pt(1)/reproj_pt(3),reproj_pt(2)/reproj_pt(3)];
% 
% %testin = intrinsic_matrix*(test_rotation*transpose(M_i(1,:)) + transpose(test_translation));
% % testin = intrinsic_matrix*(worldOrientation*transpose(M_i(1,:)) + transpose(translation_matrix));
% % testin = testin / testin(3);
% 
% figure(3), imshow(I);
% hold on;
% plot(reproj_cm_pt(1),reproj_cm_pt(2),'ro');

%kr_inverse = inv(rotation_matrix*(camera_m.IntrinsicMatrix'));

%Image_Location = zeros(6,3);
% World_Location = zeros(15,3);
% 
% for i = 1:15
%     
%     %Image_Location(i,:) = imagePoints(i,:);
%     Image_Location = [imagePoints(i,1);imagePoints(i,2);1];
%     %World_Location(i,:) = transpose( inv(p1) * (kinv * Image_Location ) + transpose(t1));
%     
%     World_Location(i,:) = inv(rotation_matrix)*(inv(intrinsic_matrix) * Image_Location) + translation_matrix;    
% end
% 
% disp(World_Location);
% 
% figure;
% pcshow(worldPoints,[1 0 0],'MarkerSize',15);
% hold on
% %pcshow(World_Location,'VerticalAxis','Y','VerticalAxisDir','down','MarkerSize',1);
% pcshow(World_Location,[1 0 1],'Marker',15);
% %plot3(sift_point_world(:,1),sift_point_world(:,2),sift_point_world(:,3),'o');
% plotCamera('Size',1/20,'Orientation',worldOrientation,'Location',worldLocation);
% plot3([worldLocation(1) World_Location(1,1)],[worldLocation(2) World_Location(1,2)],[worldLocation(3) World_Location(1,3)],'b');
% %hold off

%sift feature calculation
[frames1, descrs1] = getFeatures(I, 'peakThreshold', 0.01) ;

xq = frames1(1,:);
yq = frames1(2,:);

indices = zeros(numel(xq),1);
counter_sift = 0;
counter_sift_point = 0;

World_location_sift_1 = zeros(numel(xq),3);

% %disp(frames1);
% 
%faceid -> doing it for only 1 face

% %faceid
% counter_faces = 1;
% matr_faceid = zeros(6,4);
% for i = 1:6
%     fa = total_faces(i);
%     origin_fa = fa;
%     fa = int2str(fa);
%     
%     for j = 1:no_vertices
%         if str2num(fa(1:1)) == train_i(j,3)
%             v_1_ = j;
%         end
%         if str2num(fa(2:2)) == train_i(j,3)
%             v_2_ = j;
%         end
%         if str2num(fa(3:3)) == train_i(j,3)
%             v_3_ = j;
%         end
%         if str2num(fa(4:4)) == train_i(j,3)
%             v_4_ = j;
%         end
%     end
%     
%     if v_1_>0 && v_2_>0 && v_3_>0 && v_4_>0
% 		matr_faceid(counter_faces,1) = v_1_;%train_i(v_1_,3);
% 		matr_faceid(counter_faces,2) = v_2_;%train_i(v_2_,3);
% 		matr_faceid(counter_faces,3) = v_3_;%train_i(v_3_,3);
% 		matr_faceid(counter_faces,4) = v_4_;%train_i(v_4_,3);
% 		counter_faces = counter_faces + 1;
%     end
%     v_1_ = 0;
%     v_2_ = 0;
%     v_3_ = 0;
%     v_4_ = 0;
% end

%faceid
counter_faces = 1;
matr_faceid = zeros(12,3);
for i = 1:12
    
    fa = total_triangle_faces(i);
    fa = int2str(fa);
    
    for j = 1:no_vertices
        if str2num(fa(1:1)) == train_i(j,3)
            v_1_ = j;
        end
        if str2num(fa(2:2)) == train_i(j,3)
            v_2_ = j;
        end
        if str2num(fa(3:3)) == train_i(j,3)
            v_3_ = j;
        end
    end
    
    if v_1_>0 && v_2_>0 && v_3_>0
		matr_faceid(counter_faces,1) = v_1_;%train_i(v_1_,3);
		matr_faceid(counter_faces,2) = v_2_;%train_i(v_2_,3);
		matr_faceid(counter_faces,3) = v_3_;%train_i(v_3_,3);
		counter_faces = counter_faces + 1;
    end
    v_1_ = 0;
    v_2_ = 0;
    v_3_ = 0;
end


World_Location = zeros(15,3);

for i = 1:numel(imagePoints(:,2))
    
    %Image_Location(i,:) = imagePoints(i,:);
    Image_Location = [imagePoints(i,1);imagePoints(i,2);1];
    %World_Location(i,:) = transpose( inv(p1) * (kinv * Image_Location ) + transpose(t1));
    
    World_Location(i,:) = inv(rotation_matrix)*(inv(intrinsic_matrix) * Image_Location) + translation_matrix;    
end

figure(2);
pcshow(worldPoints,[1 0 0],'MarkerSize',15);
hold on
pcshow(World_Location,[0 0 0],'Marker',15);
plotCamera('Size',1/20,'Orientation',worldOrientation,'Location',worldLocation);


%looping through faceid
fff = 0;
for p=1:(counter_faces-1)
% Define a square region with a square hole. Specify the vertices of the
% outer loop in a counterclockwise direction, and specify the vertices for
% the inner loop in a clockwise direction. Use |NaN| to separate the
% coordinates for the outer and inner loops.

%xv = [train_i(matr_faceid(1,1),1) train_i(matr_faceid(1,2),1) train_i(matr_faceid(1,3),1) train_i(matr_faceid(1,4),1) train_i(matr_faceid(1,1),1)];
%yv = [train_i(matr_faceid(1,1),2) train_i(matr_faceid(1,2),2) train_i(matr_faceid(1,3),2) train_i(matr_faceid(1,4),2) train_i(matr_faceid(1,1),2)];

xv = [train_i(matr_faceid(p,1),1) train_i(matr_faceid(p,2),1) train_i(matr_faceid(p,3),1) train_i(matr_faceid(p,1),1)];
yv = [train_i(matr_faceid(p,1),2) train_i(matr_faceid(p,2),2) train_i(matr_faceid(p,3),2) train_i(matr_faceid(p,1),2)];

%xv = xv/2;
%yv = yv/2;

%xq = frames1(1,:);
%yq = frames1(2,:);

% Determine whether each point lies inside or on the edge of the polygon area.
[in,on] = inpolygon(xq,yq,xv,yv);

%disp(xq(in));

% Plot the polygon and the query points. Display the points inside the
% polygon with a red plus. Display the points outside the polygon with a
% blue circle.
%figure

%numel(xq(in));
%numel(xq(on));
figure(1);
plot(xv,yv,'LineWidth',2) % polygon
%hold on
plot(xq(in),yq(in),'r+') % points inside

sift_point_x = xq(in);
sift_point_y = yq(in);
no_sift_points = numel(xq(in));
% fff = fff + no_sift_points;
% disp('sift_point is');
% disp(sift_point_x(1));
%indices = zeros(counter_faces-1,no_sift_points);

% for q=1:no_sift_points
%    
%     for w=1:numel(xq)
%         
%         if (xq(w) == sift_point_x(q)) && (yq(w) == sift_point_y(q))
%             indices(counter_sift) = w;
%             counter_sift = counter_sift + 1;
%             break;
%         end;
%         
%     end;
%     
% end;

for i=1:numel(xq)
    if in(i) == 1
        counter_sift = counter_sift + 1;
        indices(counter_sift) = i;
    end;
end;

%sift_point_z = 1;
World_Location_sift = zeros(no_sift_points,3);
% 
for i = 1:no_sift_points
    
    %Image_Location(i,:) = imagePoints(i,:);
    Image_Location = [sift_point_x(i);sift_point_y(i);1];
    %World_Location_sift(i,:) = transpose( inv(p1) * (kinv * Image_Location ) + transpose(t1));
    
    World_Location_sift(i,:) = inv(rotation_matrix)*(inv(intrinsic_matrix) * Image_Location) + translation_matrix;    
end

% World_Location = zeros(15,3);
% 
% for i = 1:15
%     
%     %Image_Location(i,:) = imagePoints(i,:);
%     Image_Location = [imagePoints(i,1);imagePoints(i,2);1];
%     %World_Location(i,:) = transpose( inv(p1) * (kinv * Image_Location ) + transpose(t1));
%     
%     World_Location(i,:) = inv(rotation_matrix)*(inv(intrinsic_matrix) * Image_Location) + translation_matrix;    
% end

disp(World_Location_sift);

figure(2);
% pcshow(worldPoints,[1 0 0],'MarkerSize',15);
% hold on
%pcshow(World_Location,'VerticalAxis','Y','VerticalAxisDir','down','MarkerSize',1);
pcshow(World_Location_sift,[1 0 1],'Marker',15);
%pcshow(World_Location,[0 0 0],'Marker',15);

%plotting the bounding box of a face of 3d object in world frame

object3d_x = [M_i(train_i(matr_faceid(p,1),3),1) M_i(train_i(matr_faceid(p,2),3),1) M_i(train_i(matr_faceid(p,3),3),1) M_i(train_i(matr_faceid(p,1),3),1)];
object3d_y = [M_i(train_i(matr_faceid(p,1),3),2) M_i(train_i(matr_faceid(p,2),3),2) M_i(train_i(matr_faceid(p,3),3),2) M_i(train_i(matr_faceid(p,1),3),2)];
object3d_z = [M_i(train_i(matr_faceid(p,1),3),3) M_i(train_i(matr_faceid(p,2),3),3) M_i(train_i(matr_faceid(p,3),3),3) M_i(train_i(matr_faceid(p,1),3),3)];

plot3(object3d_x,object3d_y,object3d_z,'LineWidth',2); % polygon

%plotting the bounding box of a face of 3d object in camera frame

% object3d_x = [M_i(train_i(matr_faceid(1,1),3),1) M_i(train_i(matr_faceid(1,2),3),1) M_i(train_i(matr_faceid(1,3),3),1) M_i(train_i(matr_faceid(1,4),3),1) M_i(train_i(matr_faceid(1,1),3),1)];
% object3d_y = [M_i(train_i(matr_faceid(1,1),3),2) M_i(train_i(matr_faceid(1,2),3),2) M_i(train_i(matr_faceid(1,3),3),2) M_i(train_i(matr_faceid(1,4),3),2) M_i(train_i(matr_faceid(1,1),3),2)];
% object3d_z = [M_i(train_i(matr_faceid(1,1),3),3) M_i(train_i(matr_faceid(1,2),3),3) M_i(train_i(matr_faceid(1,3),3),3) M_i(train_i(matr_faceid(1,4),3),3) M_i(train_i(matr_faceid(1,1),3),3)];
% 
% plot3(object_x,object_y,object_z,'LineWidth',2) % polygon

%plot3(sift_point_world(:,1),sift_point_world(:,2),sift_point_world(:,3),'o');
%plotCamera('Size',1/20,'Orientation',worldOrientation,'Location',worldLocation);
%plot3([worldLocation(1) World_Location(1,1)],[worldLocation(2) World_Location(1,2)],[worldLocation(3) World_Location(1,3)],'b');
just = randi(no_sift_points);
plot3([worldLocation(1) World_Location_sift(just,1)],[worldLocation(2) World_Location_sift(just,2)],[worldLocation(3) World_Location_sift(just,3)],'b');
%hold off

orig = worldLocation;
vert0 = M_i(train_i(matr_faceid(p,1),3),:); 
vert1 = M_i(train_i(matr_faceid(p,2),3),:);
vert2 = M_i(train_i(matr_faceid(p,3),3),:);

for i = 1:no_sift_points
dir = World_Location_sift(i,:) - worldLocation;
[intersect, t, u, v, xcoor] = TriangleRayIntersection(orig, dir, vert0, vert1, vert2);
plot3(xcoor(1),xcoor(2),xcoor(3),'bo'); % polygon
counter_sift_point = counter_sift_point + 1;
World_location_sift_1(counter_sift_point,:) = xcoor;
end;

end;
% disp(u);
% disp(v);
% disp(xcoor);
% disp(indices);

sift_image8 = indices(1:counter_sift); %it contains the index of sift points
sift_location_image8 = zeros(counter_sift,3);
descriptor_matrix_image8 = zeros(numel(descrs1(:,1)),counter_sift); %(dimension,no_of_sift_points)

for i=1:counter_sift
    sift_location_image8(i,:) = World_location_sift_1(i,:);
    descriptor_matrix_image8(:,i) = descrs1(:,sift_image8(i));
end;



%figure(2);
%pcshow(M_i,[1 0 1],'Marker',15);
%hold on;
%for i=1:counter_sift
%plot3(sift_location_image1(i,1),sift_location_image1(i,2),sift_location_image1(i,3),'r+');
%end
% 
% %equation of a plane
% 
% %Vector p0_p1;
% %cv::Point3f p0_p1;
% %p0_p1.x = p0.x - p1.x;
% %p0_p1.y = p0.y - p1.y;
% %p0_p1.z = p0.z - p1.z;
% 
% %Vector p0_p2;
% %cv::Point3f p0_p2;
% %p0_p2.x = p0.x - p2.x;
% %p0_p2.y = p0.y - p2.y;
% %p0_p2.z = p0.z - p2.z;
% 
% p0_p1_x = M_i(train_i(matr_faceid(1,2),3),1) - M_i(train_i(matr_faceid(1,1),3),1);
% p0_p1_y = M_i(train_i(matr_faceid(1,2),3),2) - M_i(train_i(matr_faceid(1,1),3),2);
% p0_p1_z = M_i(train_i(matr_faceid(1,2),3),3) - M_i(train_i(matr_faceid(1,1),3),3);
% 
% p0_p2_x = M_i(train_i(matr_faceid(1,3),3),1) - M_i(train_i(matr_faceid(1,1),3),1);
% p0_p2_y = M_i(train_i(matr_faceid(1,3),3),2) - M_i(train_i(matr_faceid(1,1),3),2);
% p0_p2_z = M_i(train_i(matr_faceid(1,3),3),3) - M_i(train_i(matr_faceid(1,1),3),3);
% 
% % p0_p1_x = train_i(matr_faceid(1,2),1) - train_i(matr_faceid(1,1),1);
% % p0_p1_y = train_i(matr_faceid(1,2),2) - train_i(matr_faceid(1,1),2);
% % p0_p1_z = train_i(matr_faceid(1,2),3) - train_i(matr_faceid(1,1),3);
% % 
% % p0_p2_x = train_i(matr_faceid(1,3),1) - train_i(matr_faceid(1,1),1);
% % p0_p2_y = train_i(matr_faceid(1,3),2) - train_i(matr_faceid(1,1),2);
% % p0_p2_z = train_i(matr_faceid(1,3),3) - train_i(matr_faceid(1,1),3);
% 
% p0_p1 = [p0_p1_x p0_p1_y p0_p1_z];
% p0_p2 = [p0_p2_x p0_p2_y p0_p2_z];
% %%Normal vector
% n = cross(p0_p1,p0_p2);
% 
% n = n/sqrt(n * n');
% 
% a = n(1);
% b = n(2);
% c = n(3);
% 
% %d = -(a*p0.x + b*p0.y + c*p0.z);
% %d = -(a*( + b*(M_i(train_i(matr_faceid(1,1),3),2)) + c*(M_i(train_i(matr_faceid(1,1),3),3)));
% d = - n' * M_i(train_i(matr_faceid(1,1),3),:); 
% %Camera center
% rotation_matrix_inv = (rotation_matrix)';
% C = (-rotation_matrix_inv*translation_matrix);
% 
% %scaling factor
% 
% %numinator
% 
% numinator = (-1)*(d + dot(n,C));
% 
% %intrinsic&rotation inverse
% intrinsic_matrix = [2960.37845 0 1841.68855;
%                     0 2960.37845 1235.23369;
%                     0 0 1];
% kr_inverse = inv(intrinsic_matrix*rotation_matrix);
% 
% x_cord = xq(in);
% y_cord = yq(in);
% 
% disp('x_cord_sift_points');
% disp(x_cord);
% disp(y_cord);
% 
% ptCloud = pcread('model/teabox.ply');
% figure, pcshow(ptCloud);
% hold on;
% 
% sift_point_world = zeros(numel(xq(in)),3);
% for i = 1:numel(xq(in))
% 
% % sift_point = zeros(numel(xq(in)),2);
% % sift_point(:,1) = xq(in);
% % sift_point(:,2) = yq(in);
% % sift_point(:,3) = ones(numel(xq(in)),1)/2;
% % sift_point = sift_point*2;
% 
% sift_point = [x_cord(i) y_cord(i) 1]';
% %sift_point = sift_point*2;
% 
% %denominator
% %denominator = n*(kr_inverse*sift_point.');
% denominator = dot(n,(kr_inverse*sift_point));
% w = numinator/denominator;
% 
% %Determine the number of points lying inside or on the edge of the polygon area.
% %numel(xq(in));
% 
% % C = dot(A,B)
% sift_point_world(i,:) = (w*kr_inverse*sift_point + C)'; 
% 
% %plotting sift on 3d object -> for only 1 face as of now.
% %%
% output = plot3(sift_point_world(i,1),sift_point_world(i,2),sift_point_world(i,3),'b+');
% 
% %plot3(sift_point_world(:,1),sift_point_world(:,2),sift_point_world(:,3),'o');
% end

%# make sure the image doesn't disappear if we plot something else
%hold on

%plot(xq(~in),yq(~in),'bo') % points outside
%hold off

%%
% Query points in the square hole are outside the polygon.

%% Find the corners.
% corners = detectHarrisFeatures(I);
%% Display the results.
%ptCloud = pcread('model/teabox.ply');
%pcshow(ptCloud);
%image_com = [I,I];
%figure, imshow(image_com);
%%imshow(MM);
%%plot(corners.selectStrongest(1));
%imshow(temp);
%plot(floor(x), floor(y), b);
%RGB = insertText(I,position,500,'AnchorPoint','LeftBottom');
%plot(ac_pt_x, ac_pt_y);
%plot(corners.selectStrongest(1));
%hold on;

% Define x and y coordinates of 500 random points. Initialize the
% random-number generator to make the output of |randn| repeatable.
%rng default
%xq = rand(100,1)*3;
%yq = rand(100,1)*3;

%# read and display image
%I = imread('images/init_texture/DSC_9743.JPG');
%temp = I;
%I = rgb2gray(I);

%# define points (in matrix coordinates)
%p1 = [10,100];
%p2 = [100,20];

%# plot the points.
%# Note that depending on the definition of the points,
%# you may have to swap x and y
%plot([p1(2),p2(2)],[p1(1),p2(1)],'Color','r','LineWidth',2)
%figure,imshow(I)
%vl_plotframe(frames1, 'linewidth', 2) ;
