
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Actions
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


facing(D,I+1) :- approach(D,I), door(D), I=0..n-1.
at(R2,I+1) :- approach(D,I), at(R1,I), hasdoor(R2,D), acc(R1,R2), I=0..n-1.
:- approach(D,I), facing(D,I), door(D), I=0..n.
:- approach(D,I), hasdoor(R1,D), at(R2,I), R1 != R2, not acc(R1,R2), not dooracc(R2,D,R1).


at(R2,I+1) :- gothrough(D,I),  at(R1,I), dooracc(R1,D,R2), I=0..n-1.
-facing(D,I+1) :- gothrough(D,I), I=0..n.
:- gothrough(D,I), not facing(D,I), door(D), I=0..n.
:- gothrough(D,I), not open(D,I), door(D), I=0..n.
:- gothrough(D,I), at(R,I), not hasdoor(R,D), door(D), room(R), I=0..n.


open(D,I+1) :- opendoor(D,I), door(D), I=0..n-1.
:- opendoor(D,I), -facing(D,I), door(D), I=0..n.
:- opendoor(D,I), open(D,I), door(D), I=0..n.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Static laws
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%you can't be at two places at the some time
-at(L2,I):- at(L1,I), location(L2), L1 != L2.


%you can be facing only one door at a time
-facing(D2,I):- facing(D1,I), door(D2), D1 != D2.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Inertia
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%at is inertial
at(L,I+1) :- at(L,I), not -at(L,I+1), I=0..n-1.

%facing is inertial
facing(D,I+1) :- facing(D,I), not -facing(D,I+1), I=0..n-1.
-facing(D,I+1) :- -facing(D,I), not facing(D,I+1), I=0..n-1.

%open is inertial
open(D,I+1) :- open(D,I), not -open(D,I+1), I=0..n-1.
-open(D,I+1) :- -open(D,I), not open(D,I+1), I=0..n-1.


