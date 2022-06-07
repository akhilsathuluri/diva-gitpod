close all
clc
clear all 


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
 
file = load("Samples_Mass\samples_4972_lx_190_370_ly_100_nelx_16_nely_16");


X = file.samples(1:4,:);
y = file.samples(5,:);
N = length(y);

figure(1)
subplot(2,3,1)
scatter(X(1,:),X(2,:),20,y,'filled');xlabel('k11');ylabel('k22');hold on;
plot(646.2338, 6.9424e+07,'rx');
plot(728.9041, 6.6077e+07,'rx'); 
plot(376.5532, 5.1296e+07 ,'rx'); 
plot(164.2839, 3.0817e+07,'rx'); 
plot(236.6771, 1.7341e+07 ,'rx'); 
subplot(2,3,2)
scatter(X(2,:),X(3,:),20,y,'filled');xlabel('k22');ylabel('k44'); hold on;
plot(6.9424e+07, 5.6102e+07,'rx');
plot(6.6077e+07, 5.9054e+07,'rx'); 
plot(5.1296e+07, 4.0151e+07 ,'rx'); 
plot(3.0817e+07, 1.2593e+07,'rx'); 
plot(1.7341e+07, 2.9724e+05 ,'rx'); 
subplot(2,3,4)
scatter(X(1,:),X(4,:),20,y,'filled');xlabel('k11');ylabel('lx'); hold on;
plot(646.2338, lx(1),'rx');
plot(728.9041, lx(2),'rx'); 
plot(376.5532, lx(3) ,'rx'); 
plot(164.2839, lx(4),'rx'); 
plot(236.6771, lx(5) ,'rx'); 
subplot(2,3,5)
scatter(X(2,:),X(4,:),20,y,'filled');xlabel('k22');ylabel('lx'); hold on;
plot(6.9424e+07, lx(1),'rx');
plot(6.6077e+07, lx(2),'rx'); 
plot(5.1296e+07, lx(3) ,'rx'); 
plot(3.0817e+07, lx(4),'rx'); 
plot(1.7341e+07, lx(5) ,'rx'); 
subplot(2,3,6)
scatter(X(3,:),X(4,:),20,y,'filled');xlabel('k44');ylabel('lx'); hold on 
plot(5.6102e+07,lx(1),'rx');
plot(5.9054e+07,lx(2),'rx'); 
plot(4.0151e+07 ,lx(3),'rx'); 
plot(1.2593e+07,lx(4),'rx'); 
plot(2.9724e+05 ,lx(5),'rx'); 



%% Mass

if true
%Training

rmax=0;
rmin=1; 
rmean=0; 
NN=10;

for ii=1:NN 
    r = 0.5;
    mserror = 100;
    p=max(y);
    relmaxerr = 1;

    nHl = 1;
    for i=1:5
        nN = 1;
        for j=1:10
            net = feedforwardnet([nN*ones(1,nHl)]);
            net.divideParam.trainRatio = 0.7;
            net.divideParam.valRatio = 0.15;
            net.divideParam.testRatio = 0.15;
            net.performParam.normalization = 'percent';
            [net, tr] = trainlm(net,X,y);
            [r_new,~,~] = regression(y(tr.testInd), net(X(:,tr.testInd)));
            p_new = mae(y(tr.testInd)-net(X(:,tr.testInd)));
            %mserror_new =  tr.best_tperf
            mserror_new =  mse(y(tr.testInd),net(X(:,tr.testInd)));
            relmaxerr_new = max(abs((y-net(X))./y));
           
            if r < r_new  && mserror > mserror_new
                mserror = mserror_new;
                relmaxerr =relmaxerr_new;
                r = r_new;
                p = p_new;
                result_net = net;
                results_nN = nN;
                results_nHl = nHl;
            end
            nN = nN +2;
        end 
        nHl = nHl + 1;
    end
    
    results_nN
    results_nHl
    net = result_net;
    rvalue = num2str(r^2);
    

    mserror_ = fix(mserror);
    mserror = mserror - mserror_;
    mserror = num2str(mserror); 
    mserror_ = num2str(mserror_); 
    


    i=1
    r = (lx_d(i)*ly_d(i)*lz_d(i))/(l0_d(1)*l0_d(2)*l0_d(3));
    r*net([646.2338, 6.9424e+07, 5.6102e+07,lx(i)]')
    
    
    i=2
    r = (lx_d(i)*ly_d(i)*lz_d(i))/(l0_d(1)*l0_d(2)*l0_d(3));
    r*net([728.9041, 6.6077e+07, 5.9054e+07,lx(i)]')
    
    i=3
    r = (lx_d(i)*ly_d(i)*lz_d(i))/(l0_d(1)*l0_d(2)*l0_d(3));
    r*net([376.5532, 5.1296e+07 4.0151e+07,lx(i)]')
    
    
    i=4
    r = (lx_d(i)*ly_d(i)*lz_d(i))/(l0_d(1)*l0_d(2)*l0_d(3));
    r*net([164.2839, 3.0817e+07, 1.2593e+07,lx(i)]')

    i=5
    r = (lx_d(i)*ly_d(i)*lz_d(i))/(l0_d(1)*l0_d(2)*l0_d(3));
    r*net([236.6771, 1.7341e+07, 2.9724e+05,lx(i)]')    

    save("Mass_Estimator/ANN_r2_" + "0_" + rvalue(3:end) + "_mse_" + mserror_ + "_" + mserror(3:end) + "_samples_" + num2str(N) + ...
        "_lx_" + num2str(lx_ub) + "_" + num2str(lx_lb) + "_ly_" + num2str(ly(1))+ "_lz_" + num2str(lz(1)) + "_nelx_" + num2str(nelx) + "_nely_" + num2str(nely) + "_nelz_" + + num2str(nelz),"net")

end 
    
end 
