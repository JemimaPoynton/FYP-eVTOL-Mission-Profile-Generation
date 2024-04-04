function[dw,DW]=fastdw(mesh)
one_by_four_pi=1/(4*pi);

[psize, vsize , ~]=size(mesh.nodes.xq);


%disp('running right')
%psize=size(lattice.COLLOC,1);
lemma=ones(1,psize);

LDW=zeros(psize,psize,7,3);

mCOLLOC(:,:,1)=mesh.control.xyz(:,1)*lemma;
mCOLLOC(:,:,2)=mesh.control.xyz(:,2)*lemma;
mCOLLOC(:,:,3)=mesh.control.xyz(:,3)*lemma;

mN(:,:,1)=mesh.unitVec(:,1)*lemma;
mN(:,:,2)=mesh.unitVec(:,2)*lemma;
mN(:,:,3)=mesh.unitVec(:,3)*lemma;

for j=1:(vsize-1)
    
    lr1(:,:,1)=(mesh.nodes.xq(:,j)*lemma)';
    lr1(:,:,2)=(mesh.nodes.yq(:,j)*lemma)';
    lr1(:,:,3)=(mesh.nodes.zq(:,j)*lemma)';
    
    lr2(:,:,1)=(mesh.nodes.xq(:,j+1)*lemma)';
    lr2(:,:,2)=(mesh.nodes.yq(:,j+1)*lemma)';
    lr2(:,:,3)=(mesh.nodes.zq(:,j+1)*lemma)'; 
    
    r1=lr1-mCOLLOC;
    r2=lr2-mCOLLOC;
    
    warning off
    LDW(:,:,j,:)=mega(r1,r2);
    warning on
end
LDW(find((isnan(LDW(:,:,:,:)))))=0;

DW=-squeeze(sum(LDW,3))*one_by_four_pi;

dw=sum(DW.*mN,3);
end

function[DW2]=mega(r1,r2)
%% First part
F1=cross(r1,r2,3);

LF1=(sum(F1.^2,3));



F2(:,:,1)=F1(:,:,1)./(LF1);
F2(:,:,2)=F1(:,:,2)./(LF1);
F2(:,:,3)=F1(:,:,3)./(LF1);
%clear('F1')


%% Next part

Lr1=sqrt(sum(r1.^2,3)); 
Lr2=sqrt(sum(r2.^2,3));


R1(:,:,1)=r1(:,:,1)./Lr1;
R1(:,:,2)=r1(:,:,2)./Lr1;
R1(:,:,3)=r1(:,:,3)./Lr1;

R2(:,:,1)=r2(:,:,1)./Lr2;
R2(:,:,2)=r2(:,:,2)./Lr2;
R2(:,:,3)=r2(:,:,3)./Lr2;



L1=(R2-R1);
%clear('R1','R2')



%% Third part
R0=(r2-r1);

radial_distance=sqrt((LF1./(sum(R0.^2,3))));


%% combinging 2 and 3
L2=  R0(:,:,1).*L1(:,:,1)...
    +R0(:,:,2).*L1(:,:,2)...
    +R0(:,:,3).*L1(:,:,3);



%% Downwash
DW(:,:,1)=F2(:,:,1).*L2;
DW(:,:,2)=F2(:,:,2).*L2;
DW(:,:,3)=F2(:,:,3).*L2;

DW2 = DW;


end