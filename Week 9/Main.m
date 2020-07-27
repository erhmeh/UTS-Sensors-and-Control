clc;
clear;

%%
Target = [0,0;
          800,0;
          0,800;
          800,800];
      
Obs = Target+50;

Z = 50;

f = 400;
p = 400;

Lambda = 0.1;

%%
xy = (Target-p)/f;
Obsxy = (Obs-p)/f;

%%
n = length(Target(:,1));

Lx = [];
for i=1:n;
    Lxi = FuncLx(xy(i,1),xy(i,2),Z);
    Lx = [Lx;Lxi];
end;

%%
e2 = Obsxy-xy;
e = reshape(e2',[],1);
de = -e*Lambda;

%%
Lx2 = inv(Lx'*Lx)*Lx';
Vc = -Lambda*Lx2*e
