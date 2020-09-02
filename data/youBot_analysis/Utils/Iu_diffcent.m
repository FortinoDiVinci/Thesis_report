%Iu_DIFFCENT	derivee numérique par difference centrale
%	DIFFCENT(Y, PAS), for a vector Y, is [Y(2)-Y(1) (Y(3)-Y(1))/2 ... Y(n)-Y(n-1)]/PAS.
%
%	See also DIFF, GRADIENT, DEL2, INT, SYMVAR.
%
%	Copyright (c) 1994 by M. Gautier, LAN Robotique
%	Exemple : yd=diffcent(y,pas);

function [yd] = Iu_diffcent(y,t)

ny=length(y);

dtemps = [(t(2)-t(1));(t(3:ny)-t(1:ny-2))/2;(t(ny)-t(ny-1))];
yd=[(y(2)-y(1));(y(3:ny)-y(1:ny- 2))/2;(y(ny)-y(ny-1))]./dtemps;