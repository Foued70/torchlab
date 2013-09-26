%Author: Frederic Richard (june, 2009)
%Decomposition of an image into a periodic component and a smooth component
%The method is described in the paper "Periodic plus smooth image
%decomposition" by Lionel Moisan, preprint 2009.


%INPUT :
%  U image to be decomposed
%OUTPUT :
%  Per : periodic component of X
%  The smooth component can be obtained as :  Smo = U - Per
%%%%%%%

function [Per]=PeriodicPlusSmoothDecomposition(U)

[M,N]=size(U);

Per=zeros([M,N]);
Per(1,:)=U(1,:)-U(M,:); 
Per(M,:)=U(M,:)-U(1,:); 
Per(:,1)=U(:,1)-U(:,N); 
Per(:,N)=U(:,N)-U(:,1);
Per(1,1)=2*U(1,1)-U(M,1)-U(1,N); 
Per(M,1)=2*U(M,1)-U(1,1)-U(M,N); 
Per(1,N)=2*U(1,N)-U(1,1)-U(M,N); 
Per(M,N)=2*U(M,N)-U(M,1)-U(1,N);


U=fft2(U); Per=fft2(Per);

Per=U-Per./((4)-2*(repmat(cos(2*pi*[0:(M-1)]'/M),1,N)+repmat(cos(2*pi*[0:(N-1)]/N),M,1)));
Per(1,1)=U(1,1);

Per = real(ifft2(Per));
