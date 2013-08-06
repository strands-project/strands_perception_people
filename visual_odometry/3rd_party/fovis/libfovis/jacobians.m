% matlab script that, at some point in time, was used to generate the Jacobians
% for reprojection.cpp

syms fx px py r p y tx ty tz ax ay az

R_r = [ 1 0 0 0 ; 0 cos(r) -sin(r) 0; 0 sin(r) cos(r) 0; 0 0 0 1 ]
R_p = [ cos(p) 0 sin(p) 0; 0 1 0 0 ; -sin(p) 0 cos(p) 0; 0 0 0 1 ]
R_y = [ cos(y), -sin(y), 0 0; sin(y) cos(y) 0 0 ; 0 0 1 0 ; 0 0 0 1 ]

dRr_dr = diff(R_r, r)
dRp_dp = diff(R_p, p)
dRy_dy = diff(R_y, y)

R = R_y * R_p * R_r

%dR_dr = diff(R, r)

T = [ 1 0 0 tx; 0 1 0 ty; 0 0 1 tz; 0 0 0 1 ]

K = [ fx 0 px 0 ; 0 fx py 0; 0 0 1 0 ]         

X = [ ax; ay; az; 1 ]

M = T * R

P = K * M * X

Tinv = [ 1 0 0 -tx; 0 1 0 -ty; 0 0 1 -tz; 0 0 0 1 ];
Rinv = R.'
Minv = R.' * Tinv

simplify(M * Minv)

P2 = simplify(K * Minv)

dP2_tx = simplify(diff(P2, tx))
dP2_ty = simplify(diff(P2, ty))
dP2_tz = simplify(diff(P2, tz))
dP2_r  = simplify(diff(P2, r))
dP2_p  = simplify(diff(P2, p))
dP2_y  = simplify(diff(P2, y))

Ju2 = [ dP2_tx(1,:) ; 
        dP2_ty(1,:) ;
        dP2_tz(1,:) ;
        dP2_r(1,:) ;
        dP2_p(1,:) ;
        dP2_y(1,:) ]

Jv2 = [ dP2_tx(2,:) ; 
        dP2_ty(2,:) ;
        dP2_tz(2,:) ;
        dP2_r(2,:) ;
        dP2_p(2,:) ;
        dP2_y(2,:) ]

Jw2 = [ dP2_tx(3,:) ; 
        dP2_ty(3,:) ;
        dP2_tz(3,:) ;
        dP2_r(3,:) ;
        dP2_p(3,:) ;
        dP2_y(3,:) ]

%u = P(1) / M(3)
%v = P(2) / M(3)
%
%du_tx = simplify(diff(u, tx))
%du_ty = simplify(diff(u, ty))
%du_tz = simplify(diff(u, tz))
%du_r = simplify(diff(u, r))
%du_p = simplify(diff(u, p))
%du_y = simplify(diff(u, y))
%
%dv_tx = simplify(diff(v, tx))
%dv_ty = simplify(diff(v, ty))
%dv_tz = simplify(diff(v, tz))
%dv_r = simplify(diff(v, r))
%dv_p = simplify(diff(v, p))
%dv_y = simplify(diff(v, y))
