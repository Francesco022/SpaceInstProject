% quest - absolute orientation with Shuster's QUEST algorithm
% QUEST stands for QUaternion ESTimator
%
% Function computes the orientation and translation for the transformation between
% two corresponding 3D point sets pi and qi so that they are related by qi = R*pi + t
% Based on the following paper:
% M.D. Shuster and S.D. Oh: "Three-Axis Attitude Determination from Vector Observations",
% Journal of Guidance and Control, Vol. 4, No. 1, January–February 1981, pp. 70–77.
%
% See also
% F.L. Markley and D. Mortari: "Quaternion Attitude Estimation Using Vector Observations",
% Journal of the Astronautical Sciences 2000, 48(2):359-380.
% and
% M.D. Shuster and Y. Cheng: "An Improvement to the QUEST Algorithm"
%
% I also have another obsolute orientation function based on the FOAM algorithm:
% https://www.mathworks.com/matlabcentral/fileexchange/63926
%
% Usage:
%    [R t err] = quest(p, q)
%
% Arguments:
%    p, q    -  3xN matrices representing N pairs of corresponding points (N>=3)
%
% Returns:
%    R       -  3x3 rotation matrix
%    t       -  3x1 3x3 rotation matrix
%    err     -  mean residual error [qi-(R*pi + t), optional]

% If you use this function in your published work, please cite it like this:
% Lourakis, M. (2017). Absolute orientation with the QUEST algorithm (http://www.mathworks.com/matlabcentral/fileexchange/65173),
% MATLAB Central File Exchange. Retrieved Nov. 14, 2017.
%
% Copyright (c) 2017 Manolis Lourakis (lourakis **at** ics forth gr)
% Institute of Computer Science,
% Foundation for Research & Technology - Hellas
% Heraklion, Crete, Greece.
% http://www.ics.forth.gr/
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
%
% The Software is provided "as is", without warranty of any kind.

function [R t err] = quest(p, q) %#ok<*NCOMMA>

% check args
if(nargout<1)
  error('At least one return argument should be specified');
end

[r1 c1]=size(p);
[r2 c2]=size(q);
if(c1~=c2)
  error('Point sets must be of the same size');
end

if(r1~=3 | r2~=3)
  error('Supplied points should be of dimension three');
end

if(c1<3)
  error('At least three point pairs needed');
end

N=c1;

% compute centroids and center points on them
pC=mean(p, 2);
qC=mean(q, 2);

pp=p - repmat(pC, 1, N);
qp=q - repmat(qC, 1, N);


B=qp*pp';

S=B+B';
z=[B(2,3)-B(3,2); B(3,1)-B(1,3); B(1,2)-B(2,1)]; % sum(cross(qp, pp), 2);
trB=trace(B);
detS=det(S);
adjS=detS*inv(S);


% lam_max is computed below from the QUEST or FOAM characteristic polynomial;
% it can also be computed like so (perhaps less efficiently):
% K=[S-eye(3)*trB z; z' trB];
% [dummy, lam]=eigs(K, 1);

if 1 % compute lam_max from the QUEST characteristic polynomial
  a=trB^2 - trace(adjS);
  b=trB^2 + z'*z;
  c=detS + z'*S*z;
  d=z'*S*S*z;
  cnst=a*b + c*trB - d;

  lam=0.5*(sum(sum(qp.^2)) + sum(sum(pp.^2))); % 0.5*(trace(qp*qp') + trace(pp*pp'));
  lamprev=0.0;

  % Newton-Raphson
  while abs((lam-lamprev)/lam) >= 1E-12
    lamprev=lam;
    lam=lam - (((lam^2 - (a+b))*lam - c)*lam + cnst)/(2*(2*lam^2 - (a+b))*lam - c);
  end
else
  % compute lam_max from the FOAM characteristic polynomial
  % (identical to that of QUEST for infinitely precise arithmetic)
  detB=det(B); %#ok<*UNRCH>
  froBsq=norm(B, 'fro')^2;
  adjB=det(B)*inv(B);
  froadjBsq=norm(adjB, 'fro')^2;

  lam=0.5*(sum(sum(qp.^2)) + sum(sum(pp.^2))); % 0.5*(trace(qp*qp') + trace(pp*pp'));
  lamprev=0.0;

  % Newton-Raphson
  while abs((lam-lamprev)/lam) >= 1E-12
    lamprev=lam;
    tmp=lam^2-froBsq;
    lam=lam - (tmp^2 - 8*lam*detB - 4*froadjBsq)/(4*tmp*lam - 8*detB);
  end
end

% compute the optimal quaternion
alpha=lam^2 - trB^2 + trace(adjS);
beta=lam - trB;
gamma=alpha*(lam + trB) - detS;

x=(alpha*eye(3) + beta*S + S*S)*z;
nrmsq=gamma^2 + x'*x;
if(nrmsq>1E-10)
  % NOTE: quat computation appears in the paper as q=1/sqrt(gamma^2 + x'*x)*[x; gamma];
  % However, the rotation has been defined to transform the reference frame to the body
  % frame, which is the inverse (i.e., R') of that we seek. Hence, we negate the scalar
  % part (and move it in the front as per our convention)
  quat=[-gamma; x]; % scalar part moved first & negated!

  % convert quaternion to rotation matrix
  if(1)
  % conversion without the need for normalization; see listing 15.3 in Watt & Watt
    qw=quat(1); qx=quat(2); qy=quat(3); qz=quat(4);
    s=2.0/nrmsq; % s=2.0/(quat'*quat);
    xs=qx*s; ys=qy*s; zs=qz*s;
    wx=qw*xs; wy=qw*ys; wz=qw*zs;
    xx=qx*xs; xy=qx*ys; xz=qx*zs;
    yy=qy*ys; yz=qy*zs; zz=qz*zs;

    R=[1.0-yy-zz, xy-wz,     xz+wy;
       xy+wz,     1.0-xx-zz, yz-wx;
       xz-wy,     yz+wx,     1.0-xx-yy];
  else
    quat=quat/norm(quat);
    R=[1-2*(quat(3)*quat(3)+quat(4)*quat(4)),  2*(quat(2)*quat(3)-quat(1)*quat(4)),   2*(quat(2)*quat(4)+quat(1)*quat(3));
       2*(quat(2)*quat(3)+quat(1)*quat(4)),  1-2*(quat(2)*quat(2)+quat(4)*quat(4)),   2*(quat(3)*quat(4)-quat(1)*quat(2));
       2*(quat(2)*quat(4)-quat(1)*quat(3)),    2*(quat(3)*quat(4)+quat(1)*quat(2)), 1-2*(quat(2)*quat(2)+quat(3)*quat(3))];
  end
else
  error(sprintf('Singular rotation! (%g)', nrmsq));
end

if(nargout>1)
  t=qC-R*pC;
end

% mean residual error
if(nargout>2)
  if 0
    err=0;
    for i=1:N
      d=q(:,i) - (R*p(:,i) + t);
      err=err + norm(d);
    end      
    err=err/N;
  end

  pp=R*p + repmat(t, 1, N);
  err=mean(sqrt((q(1,:)-pp(1,:)).^2 + (q(2,:)-pp(2,:)).^2 + (q(3,:)-pp(3,:)).^2));
end

