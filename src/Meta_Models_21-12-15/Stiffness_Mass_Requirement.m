%%% Unit System:          %%%%
%%% t, mm, N, MPa, N-mm   %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clc
clear all 
addpath('fsparse')
rng('shuffle')

%% Parameters
rmin=sqrt(3);
ncomp=5;
uc = 1;
ConstraintTolerance =1e-3;

E= 1.91e3; %MPa
rho = 1.07e-9; % t/mm^3 
mload = 5e-3; % t
g = 9.806e3; % t/s^2
Fload = 10;% N mload*g; %Load
nu = 0.36; % -
penal = 3;
r_lx = 80/180;
%Number of elements
lx=[360,281,292,337,270];
ly=[100,100,100,100,100];
lz=[50,50,50,50,50];

lx_d=[160,124.8889, 129.7778,149.7778,120];
ly_d=ly;
lz_d=lz;

nelx = 16; 
nely = 16;  
nelz = 4;

%% Stiffness Bounds 
x_lb=[];
x_ub=[];

for i=1:ncomp
    [K_rg,KE0,alldofs_g,T_rg,T_rgt]= condensationKrg(lx(i),ly(i),lz(i),r_lx,nelx,nely,nelz,ones(nely,nelz,nelx),E,nu,penal);
    x_lb = [x_lb;full(-0.05*K_rg')];
    x_ub = [x_ub;full(K_rg')];
end 
x = x_lb + (x_ub-x_lb).*rand(ncomp*3,1);

%% Load Meta Models 
file = load("Mass_Estimator/ANN_r2_0_99658_mse_0_96229_samples_4972_lx_370_190_ly_100_lz_50_nelx_16_nely_16_nelz_4.mat");
ANN = file.net;
file = load("Feasibility_Estimator/SVM_acc_0_93575_false_pos_0_043478_true_pos_0_92643_samples_4827_3330_lx_370_190_ly_100_lz_50_nelx_16_nely_16_nelz_4.mat");
SVM = file.SVM;

Kc = zeros(4,4,ncomp);
feasible = zeros(ncomp,1);
post = zeros(ncomp,1);

for n = 1:ncomp

    K_cs1_1 = x(1 + (n-1)*3);
    K_cs2_2 = x(2 + (n-1)*3);
    K_cs4_4 = x(3 + (n-1)*3);

    Kc(:,:,n) = ...
    [                                  K_cs1_1,  (K_cs1_1*lx(n)^2 + K_cs2_2 - K_cs4_4)/(2*lx(n)),                                   -K_cs1_1,  (K_cs1_1*lx(n)^2 - K_cs2_2 + K_cs4_4)/(2*lx(n))
    (K_cs1_1*lx(n)^2 + K_cs2_2 - K_cs4_4)/(2*lx(n)),                                    K_cs2_2, -(K_cs1_1*lx(n)^2 + K_cs2_2 - K_cs4_4)/(2*lx(n)),   (K_cs1_1*lx(n)^2)/2 - K_cs2_2/2 - K_cs4_4/2
                                     -K_cs1_1, -(K_cs1_1*lx(n)^2 + K_cs2_2 - K_cs4_4)/(2*lx(n)),                                    K_cs1_1, -(K_cs1_1*lx(n)^2 - K_cs2_2 + K_cs4_4)/(2*lx(n))
    (K_cs1_1*lx(n)^2 - K_cs2_2 + K_cs4_4)/(2*lx(n)),   (K_cs1_1*lx(n)^2)/2 - K_cs2_2/2 - K_cs4_4/2, -(K_cs1_1*lx(n)^2 - K_cs2_2 + K_cs4_4)/(2*lx(n)),                                    K_cs4_4];


    [Phi,Lambda] = eig(Kc(:,:,n));
    Lambda(abs(Lambda) < 1e-3) = 0; 
    [Label,~] = predict(SVM,[x(1+3*(n-1)),x(2+3*(n-1)),x(3+3*(n-1)),lx(n)]);
    feasible(n) = Label;
    if any(Lambda(:) < 0)
        feasible(n) = -1;
    end 
end 

%Displacement
K = zeros((ncomp+1)*2, (ncomp+1)*2);
D = zeros((ncomp+1)*2, 1);
F = zeros((ncomp+1)*2, 1);
for elx = 1:ncomp
    n1 =  2*(elx-1)+1; 
    n2 =  2*elx+1;
    edof = [n1;n1+1; n2; n2+1];
    K(edof,edof) = K(edof,edof) + Kc(:,:,elx);
end
F((ncomp+1)*2 -1) = Fload;
D(3:(ncomp+1)*2) = K(3:(ncomp+1)*2,3:(ncomp+1)*2)\F(3:(ncomp+1)*2);
D((ncomp+1)*2-1)

    
%Volume
V = zeros(ncomp,1);
for n=1:ncomp
        r = lx_d(n)*ly_d(n)*lz_d(n); %Actual Volume
        V(n) =  r*ANN([x(1+3*(n-1)),x(2+3*(n-1)),x(3+3*(n-1)),lx(n)]')/100; %Volume*Volumefraction
end 

%Mass
M = zeros(ncomp,1);
M = rho*V;
M



%% Functions
        
function [K_rg,KE0,alldofs_g,T_rg,T_rgt] = condensationKrg(lx,ly,lz,r_lx,nelx,nely,nelz,x,E,nu,penal)

    

    %Prepare Assembly of Stiffness Matrix
    nEl = nelx * nely * nelz;                                                  % number of elements          #3D#
    nodenrs = int32( reshape( 1 : ( 1 + nelx ) * ( 1 + nely ) * ( 1 + nelz ), ...
        1 + nely, 1 + nelz, 1 + nelx ) );                                      % nodes numbering             #3D#
    edofVec = reshape( 3 * nodenrs( 1 : nely, 1 : nelz, 1 : nelx ) + 1, nEl, 1 ); %                             #3D#
    edofMat = edofVec+int32( [0,1,2,3*(nely+1)*(nelz+1)+[0,1,2,-3,-2,-1],-3,-2,-1,3*(nely+...
       1)+[0,1,2],3*(nely+1)*(nelz+2)+[0,1,2,-3,-2,-1],3*(nely+1)+[-3,-2,-1]]);% connectivity matrix         #3D#
    nDof = ( 1 + nely ) * ( 1 + nelz ) * ( 1 + nelx ) * 3;                     % total number of DOFs        #3D#
    [ sI, sII ] = deal( [ ] );
    for j = 1 : 24
        sI = cat( 2, sI, j : 24 );
        sII = cat( 2, sII, repmat( j, 1, 24 - j + 1 ) );
    end
    [ iK , jK ] = deal( edofMat( :,  sI )', edofMat( :, sII )' );
    Iar = sort( [ iK( : ), jK( : ) ], 2, 'descend' ); clear iK jK              % reduced assembly indexing



    a = 0.5*r_lx*lx/nelx; %x
    b = 0.5*ly/nely; %y
    c = 0.5*lz/nelz; %z

    if mod(nelx,2) ~= 0 || mod(nely,2) ~= 0
        fprintf('Must have even element number! \n')
        return
    end 

    %Coordinates of the 3d elements
    coordx = -2*a*nelx/2:2*a:2*a*nelx/2; 
    coordy = 2*b*nely/2:-2*b:-2*b*nely/2;
    coordz = -2*c*nelz/2:2*c:2*c*nelz/2;
    [coordX,coordY,coordZ] = meshgrid(coordx,coordy,coordz);


    KE0 = stiffnessMatrix_brick(E, nu, 2*a, 2*b, 2*c); %full element stiffness matrix
    KE = KE0(tril(ones(length(KE0)))==1); % vector of lower triangular element stiffness matrix



    %% Stiffness Matrix
    %3d global element stiffness matrix 
    sK = reshape(KE(:)*(x(:)'.^penal),length(KE)*nelx*nely*nelz,1); 
    K = fsparse(Iar(:,1), Iar(:,2), sK, [ nDof, nDof ] );
    K = K + K' - diag( diag( K ) );
    % Guyan
    K_g = fsparse(2*3*(nely+1)*(nelz+1)+4,2*3*(nely+1)*(nelz+1)+4,0);
    [m,n]= size(K);
    K_ = sparse(m+4,n+4);
    K_(3:end-2,3:end-2) = K;
    alldofs0_g   = [1:length(K_)];
    mdofs_g = [1:(3*(nely+1)*(nelz+1))+2,length(K_)-(3*(nely+1)*(nelz+1))+1-2:length(K_)];
    sdofs_g = setdiff(alldofs0_g,mdofs_g);
    alldofs_g = [mdofs_g, sdofs_g]; 
    Kss = K_(sdofs_g,sdofs_g);
    Ksm = K_(sdofs_g,mdofs_g);
    T_g = [speye(length(mdofs_g)); -Kss\Ksm];
    T_gt = transpose(T_g);
   
    % RBE
    alldofs0_r = 1:length(K_g);                     %All dofs in original order
    sdofs_r = [3:length(K_g)-2];                    %Dofs that are to be removed
    mdofs_r = setdiff(alldofs0_r,sdofs_r);            %Dofs that remain
    alldofs_r = [mdofs_r,sdofs_r];                    %New order, sdofs are at the end
    newdofs_r = zeros(length(alldofs_r),1);           %Nonzeros will remove the condensed nodes
    newdofs_r(mdofs_r) =  1:length(mdofs_r);          %Accesing the new order with the old one
    newdofs_r(3:end-2) = 5:length(newdofs_r(5:end-2))+6;


    %Coordinates of the free nodes 
    coordRBE = [-lx/2,lx/2;
                0,0;
                0,0]; 

    C = fsparse(length(sdofs_r),length(K_g),0);   
    %% Rigid Body Left Side
    idx = 1;
    for n = 1:(nely+1)*(nelz+1)
        C(3*(n-1)+1,1) =0;                                                      % First DOF of free node, translatoric dof -> keep only y
        C(3*(n-1)+2,1) = 1;                                                     % First DOF of free node
        C(3*(n-1)+3,1) = 0;                                                     % First DOF of free node

        C_tz = cross([0;0;1],[coordX(1,1,1) - coordRBE(1,1); coordY(idx,1,1); 0]);
        C(3*(n-1)+1,2) =C_tz(1);                                                 % Second DOF of free node
        C(3*(n-1)+2,2) = C_tz(2);                                                % Second DOF of free node
        C(3*(n-1)+3,2) = C_tz(3);                                                % Second DOF of free node

        C(3*(n-1)+1,3+(n-1)*3) = -1;                                           % Slave nodes of 3d elements to be removed
        C(3*(n-1)+2,3+(n-1)*3+1) = -1;                                         % Slave nodes of 3d elements to be removed
        C(3*(n-1)+3,3+(n-1)*3+2) = -1;                                         % Slave nodes of 3d elements to be removed

        if mod(idx,(nely+1)) == 0
            idx = 1;
        else
            idx = idx+1;
        end
    end 
    %% Rigid Body Right Side
    for n = 1:(nely+1)*(nelz+1)

        C(3*(n-1)+1+(nely+1)*(nelz+1)*3,2*3*(nely+1)*(nelz+1)+3) =0;                                % First DOF of free node
        C(3*(n-1)+2+(nely+1)*(nelz+1)*3,2*3*(nely+1)*(nelz+1)+3) = 1;                                   
        C(3*(n-1)+3+(nely+1)*(nelz+1)*3,2*3*(nely+1)*(nelz+1)+3) = 0; 

        C_tz = cross([0;0;1],[coordX(1,end,1) - coordRBE(1,2); coordY(idx,1,1); 0]);
        C(3*(n-1)+1+(nely+1)*(nelz+1)*3,2*3*(nely+1)*(nelz+1)+4) =C_tz(1);                        % Second DOF of free node
        C(3*(n-1)+2+(nely+1)*(nelz+1)*3,2*3*(nely+1)*(nelz+1)+4) =C_tz(2);
        C(3*(n-1)+3+(nely+1)*(nelz+1)*3,2*3*(nely+1)*(nelz+1)+4) =C_tz(3);                             

        C(3*(n-1)+1+(nely+1)*(nelz+1)*3,end  -2 - 3*(nely+1)*(nelz+1) + 3*(n-1)+1) =-1;           % Slave nodes of 3d elements to be removed
        C(3*(n-1)+2+(nely+1)*(nelz+1)*3,end  -2 - 3*(nely+1)*(nelz+1) + 3*(n-1)+2) =-1;
        C(3*(n-1)+3+(nely+1)*(nelz+1)*3,end  -2 - 3*(nely+1)*(nelz+1) + 3*(n-1)+3) =-1;                     

        if mod(idx,(nely+1)) == 0
            idx = 1;
        else
            idx = idx+1;
        end
    end


    %Set up model for the unconstrained case
    Tsm = -C(:,sdofs_r)\C(:,mdofs_r);
    Ti = speye(length(mdofs_r)); 
    T_r = [Ti;Tsm];
    T_rt = transpose(T_r);
    T_rgt =  T_rt(1:end,newdofs_r)*T_gt;
    T_rg =  T_g*T_r(newdofs_r,1:end);
    K_rg = T_rgt*K_(alldofs_g,alldofs_g)*T_rg;   
    K_rg  = [K_rg(1,1), K_rg(2,2),K_rg(4,4)];
end 

function K = stiffnessMatrix_brick (E,nu,length_x,length_y,length_z)
% STIFFNESSMATRIX_BRICK Compute stiffness matrix for brick element
%   K = stiffnessMatrix_brick (E,nu,length_x,length_y,length_z) Computes
%   the 24x24 stiffness matrix for a regular 8 noded hexahedral finite 
%   element with Young´s modulus "E", Poisson ratio "nu" and lengths in x, 
%   y and z direction "length_x", "length_y" and "length_z" respectively.
%   Numerical integration is performed with 8 Gauss points, which yields
%   exact results for regular elements. Weight factors are one and 
%   therefore not included in the code.
%
%   Contact: Diego.Petraroia@rub.de
%
    % Compute 3D constitutive matrix (linear continuum mechanics)
    C = E./((1+nu)*(1-2*nu))*[1-nu nu nu 0 0 0; nu 1-nu nu 0 0 0;...
        nu nu 1-nu 0 0 0; 0 0 0 (1-2*nu)/2 0 0; 0 0 0 0 (1-2*nu)/2 0;...
        0 0 0 0 0 (1-2*nu)/2];
    %
    % Gauss points coordinates on each direction
    GaussPoint = [-1/sqrt(3), 1/sqrt(3)];
    %
    % Matrix of vertices coordinates. Generic element centred at the origin.
    coordinates = zeros(8,3);
    coordinates(1,:) = [-length_x/2 -length_y/2 -length_z/2];
    coordinates(2,:) = [length_x/2 -length_y/2 -length_z/2];
    coordinates(3,:) = [length_x/2 length_y/2 -length_z/2];
    coordinates(4,:) = [-length_x/2 length_y/2 -length_z/2];
    coordinates(5,:) = [-length_x/2 -length_y/2 length_z/2];
    coordinates(6,:) = [length_x/2 -length_y/2 length_z/2];
    coordinates(7,:) = [length_x/2 length_y/2 length_z/2];
    coordinates(8,:) = [-length_x/2 length_y/2 length_z/2];

    %
    % Preallocate memory for stiffness matrix
    K = zeros (24,24);
    % Loop over each Gauss point
    for xi1=GaussPoint
        for xi2=GaussPoint
            for xi3=GaussPoint
                % Compute shape functions derivatives
                dShape = (1/8)*[-(1-xi2)*(1-xi3),(1-xi2)*(1-xi3),...
                    (1+xi2)*(1-xi3),-(1+xi2)*(1-xi3),-(1-xi2)*(1+xi3),...
                    (1-xi2)*(1+xi3),(1+xi2)*(1+xi3),-(1+xi2)*(1+xi3);...
                    -(1-xi1)*(1-xi3),-(1+xi1)*(1-xi3),(1+xi1)*(1-xi3),...
                    (1-xi1)*(1-xi3),-(1-xi1)*(1+xi3),-(1+xi1)*(1+xi3),...
                    (1+xi1)*(1+xi3),(1-xi1)*(1+xi3);-(1-xi1)*(1-xi2),...
                    -(1+xi1)*(1-xi2),-(1+xi1)*(1+xi2),-(1-xi1)*(1+xi2),...
                    (1-xi1)*(1-xi2),(1+xi1)*(1-xi2),(1+xi1)*(1+xi2),...
                    (1-xi1)*(1+xi2)];
                % Compute Jacobian matrix
                JacobianMatrix = dShape*coordinates;
                % Compute auxiliar matrix for construction of B-Operator
                auxiliar = inv(JacobianMatrix)*dShape;
                % Preallocate memory for B-Operator
                B = zeros(6,24);
                % Construct first three rows
                for i=1:3
                    for j=0:7
                        B(i,3*j+1+(i-1)) = auxiliar(i,j+1);
                    end
                end
                % Construct fourth row
                for j=0:7
                    B(4,3*j+1) = auxiliar(2,j+1);
                end
                for j=0:7
                    B(4,3*j+2) = auxiliar(1,j+1);
                end
                % Construct fifth row
                for j=0:7
                    B(5,3*j+3) = auxiliar(2,j+1);
                end
                for j=0:7
                    B(5,3*j+2) = auxiliar(3,j+1);
                end
                % Construct sixth row
                for j=0:7
                    B(6,3*j+1) = auxiliar(3,j+1);
                end
                for j=0:7
                    B(6,3*j+3) = auxiliar(1,j+1);
                end

                % Add to stiffness matrix
                K = K + B'*C*B*det(JacobianMatrix);
            end
        end
    end
end
