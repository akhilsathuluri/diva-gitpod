close all
clc
clear all 

% Constants
%Number of elements
lx_lb = 190;
lx_ub = 370;

l0 = [225;120;20];
l0_d =[100;120;20];

lx=[360,281,292,337,270];
ly=[100,100,100,100,100];
lz=[50,50,50,50,50];

lx_d=[160,124.8889, 129.7778,149.7778,120];
ly_d=ly;
lz_d=lz;

nelx = 16; 
nely = 16;  
nelz = 4;



if true

%Load training data
file = load("samples_4827_lx_190_370_ly_100_nelx_16_nely_16_nelz_4");
% input-output for training
X = file.samples_class(1:4,:)';
y = file.samples_class(5,:)';
feasible = y==1; 
infeasible = not(feasible);

% info
N = length(y); 
N_f = sum(y==1);

% boolean operation - not needed - based on a balanced training data
X_f = X(logical(feasible),:); y_f= ones(length(X_f),1);
X_i = X(logical(infeasible),:); y_i= -1*ones(length(X_i),1);

% for plotting
[idx1,idx2] = dividerand(length(y),0.5,0.5,0);
X_plot = X([idx1,idx2],:);
y_plot = y([idx1,idx2]);

figure(100)
subplot(2,3,1)
scatter(X_plot(:,1),X_plot(:,2),20,y_plot,'filled');xlabel('k11');ylabel('k22');hold on;
subplot(2,3,2)
scatter(X_plot(:,2),X_plot(:,3),20,y_plot,'filled');xlabel('k22');ylabel('k44'); hold on;
subplot(2,3,4)
scatter(X_plot(:,1),X_plot(:,4),20,y_plot,'filled');xlabel('k11');ylabel('lx');hold on;
subplot(2,3,5)
scatter(X_plot(:,2),X_plot(:,4),20,y_plot,'filled');xlabel('k22');ylabel('lx'); hold on;
subplot(2,3,6)
scatter(X_plot(:,3),X_plot(:,4),20,y_plot,'filled');xlabel('k44');ylabel('lx');hold on 
% --------------------

% cost for svm  - penalities
C = zeros(2,2);
C(1,2) = 3;
C(2,1) = 1;

iter = 1; loop = 1;
name = strings(8,1);
false_pos_list = [];
true_pos_list = [];

% iterate through the cost to find optimal cost vs accuracy/false positives 
while iter <  11
    %Train Classifier
    %ClassificationCosts
    if loop > 5
        loop = 1;
        C(1,2) = C(1,2) + 1;
    end 

    [idx_f_train,idx_f_test]= dividerand(length(y_f),0.8,0.2,0);
    [idx_i_train,idx_i_test]= dividerand(length(y_i),0.8,0.2,0);

    X_train = [X_f(idx_f_train,:);X_i(idx_i_train,:)];
    y_train = [y_f(idx_f_train);y_i(idx_i_train)];

    X_test = [X_f(idx_f_test,:);X_i(idx_i_test,:)];
    y_test = [y_f(idx_f_test);y_i(idx_i_test)];
    feasible_test = (y_test==1);
    infeasible_test = not(feasible_test);


    SVM = fitcsvm(X_train,y_train,'Standardize',true,'KernelFunction','gaussian','Cost',C,'OptimizeHyperparameters','auto',...
           'HyperparameterOptimizationOptions',struct('UseParallel',true,'MaxObjectiveEvaluations',100));  
    gcf; close;
    gcf; close;


    [label2,PostProbs] = predict(SVM,X_test);
    feasible_pred = label2==1;
    infeasible_pred = not(feasible_pred);
    acc = sum(feasible_pred == feasible_test)/length(feasible_test);
    true_neg = sum(infeasible_pred & infeasible_test)/sum(infeasible_test);
    false_pos = 1 - true_neg;
    true_pos =sum(feasible_pred & feasible_test)/sum(feasible_test);

    false_pos_list = [false_pos_list,false_pos];
    true_pos_list = [true_pos_list,true_pos];


    if false_pos < 0.05 && ~any(strcmp(name,num2str(false_pos))) && true_pos > 0.85


        i=1
        predict(SVM,[646.2338, 6.9424e+07, 5.6102e+07,lx(i)])


        i=2
        predict(SVM,[728.9041, 6.6077e+07, 5.9054e+07,lx(i)])

        i=3
        predict(SVM,[376.5532, 5.1296e+07 4.0151e+07,lx(i)])


        i=4
         predict(SVM,[164.2839, 3.0817e+07, 1.2593e+07,lx(i)])

        i=5
         predict(SVM,[236.6771, 1.7341e+07, 2.9724e+05,lx(i)])    


        false_pos = num2str(false_pos);
        acc = num2str(acc);
        true_pos = num2str(true_pos);
        name(iter,1) = false_pos;
        save("Feasibility_Estimator/SVM_" + "acc_0_" + acc(3:end) +  "_false_pos_0_" + false_pos(3:end)  +  "_true_pos_0_" + true_pos(3:end) + ...
            "_samples_" + num2str(N) + "_" + num2str(N_f) + "_lx_" + num2str(lx_ub) + "_" + num2str(lx_lb) + "_ly_" + num2str(ly(1))+ "_lz_" + num2str(lz(1))+ "_nelx_" + num2str(nelx) + "_nely_" + num2str(nely)+ "_nelz_" + + num2str(nelz),"SVM")
        iter = iter+1;
    end 

    loop = loop +1;

    end
end 
