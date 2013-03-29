function [pol, invpol, errsq, maxerr, inverrsq, invmaxerr, rho, theta, XRealCord, YRealCord, ZRealCord, XImg, YImg]=findrho2theta(ocam_model, RRfin, Xt, Yt, Xp_abs, Yp_abs, ima_proc)

    xc=ocam_model.xc;
    yc=ocam_model.yc;
    ss=ocam_model.ss;
    rho=[];
    theta=[];
    XRealCord=[];
    YRealCord=[];
    ZRealCord=[];
    XImg=[];
    YImg=[];

    for kk = ima_proc,
        M=[Xt';Yt';ones(size(Xt'))];
        Mc=RRfin(:,:,kk)*M;
        rhotmp=sqrt((Xp_abs(:,:,kk)-xc).^2+(Yp_abs(:,:,kk)-yc).^2);
        radtmp=sqrt(Mc(1,:).^2+Mc(2,:).^2);
        thetatmp=atan(-radtmp./Mc(3,:));
        thetatmp(find(thetatmp<0))=thetatmp(find(thetatmp<0))+pi;
        thetatmp(find(thetatmp>pi))=thetatmp(find(thetatmp>pi))-pi;
        %thetatmp(find(thetatmp>pi/2))=pi-thetatmp(find(thetatmp>pi/2));
        theta=[theta, thetatmp];
        rho=[rho,rhotmp'];
        XRealCord=[XRealCord, Mc(1,:)];
        YRealCord=[YRealCord, Mc(2,:)];
        ZRealCord=[ZRealCord, Mc(3,:)];
        XImg=[XImg, Xp_abs(:,:,kk)'];
        YImg=[YImg, Yp_abs(:,:,kk)'];
    end;

    maxerr=inf;
    N=1;
    minerr=inf;
    pol=[];
    errsq=inf;
    while maxerr > 0.01 %& N < 5;
        N=N+1;
        [curpol,err]=fitanderr(rho, theta, N);
        maxerr=max(err);
        if maxerr < minerr,
            minerr=maxerr;
            pol=curpol;
            errsq=sum(err.^2);
        end;
    end;
    
    invmaxerr=inf;
    N=1;
    minerr=inf;
    invpol=[];
    inverrsq=inf;
    while invmaxerr > 0.01 %& N < 5;
        N=N+1;
        [curinvpol,inverr]=fitanderr(theta, rho, N);
        invmaxerr=max(inverr);
        if invmaxerr < minerr,
            minerr=invmaxerr;
            invpol=curinvpol;
            inverrsq=sum(inverr.^2);
        end;
    end;


function [pol, err]=fitanderr(x, y, N)
    pol=polyfit(x, y, N);
    err=abs(y-polyval(pol, x))/max(y);
    