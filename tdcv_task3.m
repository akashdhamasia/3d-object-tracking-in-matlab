% to compute energy function, to compute jacobian and to make the loop.

load('testingimage1');

% load('testingimage2');
% load('testingimage3');
% load('testingimage4');
% load('testingimage5');

M_i = [0 0.063 0.093;
       0.165 0.063 0.093;
       0.165 0 0.093;
       0 0 0.093;
       0 0.063 0;
       0.165 0.063 0;
       0.165 0 0;
       0 0 0];
   
I = imread('images/detection/DSC_9751.JPG');
%original_I = I;
I = rgb2gray(I);
camera_m = cameraIntrinsics([2960.37845,2960.37845],[1841.68855,1235.23369],size(I));
 figure(1), imshow(I);
 hold on;
intrinsic_matrix = [2960.37845 0 1841.68855;
                    0 2960.37845 1235.23369;
                    0 0 1];
                
%Projecting 3d sift matched point after finding R and T.

[worldR_new,worldT_new] = estimateWorldCameraPose(Image_Pts_new,World_Pts_new,camera_m);
    
[extrinsic_rotation_new,extrinsic_translation_new] = cameraPoseToExtrinsics(worldR_new,worldT_new);

R_T=[rotationMatrixToVector(extrinsic_rotation_new),extrinsic_translation_new];
R = rotationVectorToMatrix(R_T(1:3));
T = R_T(4:6)';
R_exp = rotationMatrixToVector(extrinsic_rotation_new);

p1transpose_new=transpose(extrinsic_rotation_new);
p134_new=[p1transpose_new(:,1:3) transpose(extrinsic_translation_new)];
projected2dM_new=single.empty;
projected2dM_new1 = single.empty;
for i=1:numel(World_Pts_new(:,1)) 
reproj_pt = intrinsic_matrix*p134_new*transpose([World_Pts_new(i,:) 1]);
projected2dM_new = [projected2dM_new;[reproj_pt(1)/reproj_pt(3),reproj_pt(2)/reproj_pt(3)]];
projected2dM_new1 = [projected2dM_new1;[reproj_pt(1),reproj_pt(2),reproj_pt(3)]];
end;

%plot(projected2dM_new(:,1),projected2dM_new(:,2),'b+'); 

%Intial Parameters
no_iterat=100;
lambda=0.001;
tau = 0.001;
threshold = tau + 1;
no_points=size(projected2dM_new,1);
current_loop = 1;

I = [1 0 0; 0 1 0; 0 0 1];

while current_loop <= no_iterat && threshold > tau
    
        %calculating jacobian        
        dR_r1 = ((R_exp(1) * skew(R_exp) + skew(cross(R_exp,(I-R)*[1;0;0])))/norm(R_exp,2))*R;
        dR_r2 = ((R_exp(2) * skew(R_exp) + skew(cross(R_exp,(I-R)*[0;1;0])))/norm(R_exp,2))*R;
        dR_r3 = ((R_exp(3) * skew(R_exp) + skew(cross(R_exp,(I-R)*[0;0;1])))/norm(R_exp,2))*R;

        for i = 1:no_points
            
            dRr_M1 = dR_r1 * World_Pts_new(i,:).';
            dRr_M2 = dR_r2 * World_Pts_new(i,:).';
            dRr_M3 = dR_r3 * World_Pts_new(i,:).';
            
            dMcam_p = [dRr_M1,dRr_M2,dRr_M3,I];

            dm_m = [1/projected2dM_new1(i,3), 0, -projected2dM_new1(i,1)/(projected2dM_new1(i,3)^2); ...
                  0, 1/projected2dM_new1(i,3), -projected2dM_new1(i,2)/(projected2dM_new1(i,3)^2)];
     
             J((2*i-1):(2*i),:) = dm_m * intrinsic_matrix * dMcam_p;
             
        end
        
        %calculate energy
        e_diff = (projected2dM_new -Image_Pts_new);
        e=[];
        for i=1:numel(e_diff(:,1))
        norm_e = sqrt(e_diff(i,1)*e_diff(i,1) + e_diff(i,2)*e_diff(i,2)); 
        e(2*i-1) = norm_e/2;
        e(2*i) = norm_e/2; % not quite sure separating both dimensions is legit ...
        end
        
        Delta = -inv(J' * J + lambda * eye(6))*(J' * e');
        RTnew = R_T + Delta';
        Rnew = rotationVectorToMatrix(RTnew(1:3));
        Tnew = RTnew(4:6)';
        
        p134_new=[Rnew Tnew];
        projected2dM_new2=single.empty;
        projected2dM_new3=single.empty;
        
        for i=1:numel(World_Pts_new(:,1)) 
        reproj_pt = intrinsic_matrix*p134_new*transpose([World_Pts_new(i,:) 1]);
        projected2dM_new2 = [projected2dM_new2;[reproj_pt(1)/reproj_pt(3),reproj_pt(2)/reproj_pt(3)]];
        projected2dM_new3 = [projected2dM_new3;[reproj_pt(1),reproj_pt(2),reproj_pt(3)]];
        end;
        
        %calculate new energy
        e_diff_new = (projected2dM_new(:,1:2)-Image_Pts_new(:,1:2));
        
        e_new=[];
        for i=1:numel(e_diff_new(:,1))
        norm_e = sqrt(e_diff_new(i,1)*e_diff_new(i,1) + e_diff_new(i,2)*e_diff_new(i,2)); 
        e_new(2*i-1) = norm_e/2;
        e_new(2*i) = norm_e/2; % not quite sure separating both dimensions is legit ...
        end
        
        %% update lambda
        if norm(e_new,1) > norm(e, 1)
            lambda = 10 * lambda;

            %disp(RTnew)
            %disp(sum(enew))
        else
            lambda = 0.1 * lambda;
            R_T=RTnew;
            R=Rnew;
            T=Tnew;
            e=e_new;
            final_RT = [Rnew Tnew];
        end

        threshold = norm(Delta);
        current_loop = current_loop + 1;
        
end

R_T_final = final_RT;

p134_new=R_T_final;
projected2d_final=single.empty;

for i=1:8  
reproj_pt = intrinsic_matrix*p134_new*transpose([M_i(i,:) 1]);
projected2d_final = [projected2d_final;[reproj_pt(1)/reproj_pt(3),reproj_pt(2)/reproj_pt(3)]];
end;

plot(projected2d_final(:,1),projected2d_final(:,2),'b+'); 

total_faces = [1432 5876 1584 2376 4873 1562];

xv = [projected2d_final(1,1) projected2d_final(4,1) projected2d_final(3,1) projected2d_final(2,1) projected2d_final(1,1)];
yv = [projected2d_final(1,2) projected2d_final(4,2) projected2d_final(3,2) projected2d_final(2,2) projected2d_final(1,2)];

plot(xv,yv,'LineWidth',2) % polygon

xv = [projected2d_final(5,1) projected2d_final(8,1) projected2d_final(7,1) projected2d_final(6,1) projected2d_final(5,1)];
yv = [projected2d_final(5,2) projected2d_final(8,2) projected2d_final(7,2) projected2d_final(6,2) projected2d_final(5,2)];

plot(xv,yv,'LineWidth',2) % polygon

xv = [projected2d_final(1,1) projected2d_final(5,1) projected2d_final(8,1) projected2d_final(4,1) projected2d_final(1,1)];
yv = [projected2d_final(1,2) projected2d_final(5,2) projected2d_final(8,2) projected2d_final(4,2) projected2d_final(1,2)];

plot(xv,yv,'LineWidth',2) % polygon

xv = [projected2d_final(2,1) projected2d_final(3,1) projected2d_final(7,1) projected2d_final(6,1) projected2d_final(2,1)];
yv = [projected2d_final(2,2) projected2d_final(3,2) projected2d_final(7,2) projected2d_final(6,2) projected2d_final(2,2)];

plot(xv,yv,'LineWidth',2) % polygon

xv = [projected2d_final(4,1) projected2d_final(8,1) projected2d_final(7,1) projected2d_final(3,1) projected2d_final(4,1)];
yv = [projected2d_final(4,2) projected2d_final(8,2) projected2d_final(7,2) projected2d_final(3,2) projected2d_final(4,2)];

plot(xv,yv,'LineWidth',2) % polygon

xv = [projected2d_final(1,1) projected2d_final(5,1) projected2d_final(6,1) projected2d_final(2,1) projected2d_final(1,1)];
yv = [projected2d_final(1,2) projected2d_final(5,2) projected2d_final(6,2) projected2d_final(2,2) projected2d_final(1,2)];

plot(xv,yv,'LineWidth',2) % polygon


function [skew_] = skew(a)
    skew_ = [0    -a(3) a(2);...
            a(3)  0   -a(1);...
           -a(2)  a(1) 0];
end       