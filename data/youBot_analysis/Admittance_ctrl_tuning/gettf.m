function [a2,b2,c2,d2]=gettf(p1,p2,p3)
%
% [a2,b2,c2,d2]=gettf(p1,p2,p3)
% Détermine le modèle entre le vecteur d'entrée p3=[ui:un]
% et le vecteur de sortie p2=[y1:yn] dans le schéma simulink p1
%
if isempty(p2) | isempty(p3)
  a2=[];b2=[];c2=[];d2=[];
  return
end
[a,b,c,d]=linmod(p1);
b=b(:,p3);
c=c(p2,:);
d=d(p2,p3);
n=max(size(a));
x=~(max(abs([b b]'))==0)';
as=abs(sign(a));
for i=1:(n+1);
  x=abs(sign(as*x+x));
end
xne    =~x';
if any(xne) & (nargin==0)
  disp(sprp3tf('Il y a %g etats non excite. Les etats :',sum(xne)))
  disp(fp3d(xne==1))
end
x=~(max(abs([c' c']'))==0)';
as=abs(sign(a'));
for i=1:(n+1);
  x=abs(sign(as*x+x));
end
xnv  =~x';
if any(xnv) & (nargin==0)
  disp(sprp3tf('Il y a %g etats non visible. Les etats :',sum(xnv)))
  disp(fp3d(xnv==1))
end
etat_=xne | xnv ;
b2=b(~etat_,:);
a2=a(~etat_,~etat_);
c2=c(:,~etat_);
d2=d;
if isempty(a2)
  a2=0;b2=0*d(1,:);c2=0*d(:,1);
end
return
end