function J = Jacobi(a1, a2, a3, a4, a5, a6)
T01=[cos(a1),-sin(a1),0,0;sin(a1),cos(a1),0,0;0,0,1,0.284;0,0,0,1];
T12=[-sin(a2),-cos(a2),0,0;0,0,-1,0;cos(a2),-sin(a2),0,0;0,0,0,1];
T23=[cos(a3),-sin(a3),0,9/40;sin(a3),cos(a3),0,0;0,0,1,0;0,0,0,1];
T34=[cos(a4),-sin(a4),0,0;0,0,-1,-0.2289;sin(a4),cos(a4),0,0;0,0,0,1];
T45=[sin(a5),cos(a5),0,0;0,0,1,0;cos(a5),-sin(a5),0,0;0,0,0,1];
T56=[cos(a6),-sin(a6),0,0;0,0,-1,-0.055;sin(a6),cos(a6),0,0;0,0,0,1];
% T6=tff(-pi/2,0,0,0);
T02=T01*T12;
T03=T02*T23;
T04=T03*T34;
T05=T04*T45;
T06=T05*T56;
Z1 = T01(1:3,3);
Z2 = T02(1:3,3);
Z3 = T03(1:3,3);
Z4 = T04(1:3,3);
Z5 = T05(1:3,3);
Z6 = T06(1:3,3);

P1 = T01(1:3,4);
P2 = T02(1:3,4);
P3 = T03(1:3,4);
P4 = T04(1:3,4);
P5 = T05(1:3,4);
P6 = T06(1:3,4);

J = [cross(Z1,P6 - P1),cross(Z2,P6 - P2),cross(Z3,P6 - P3),cross(Z4,P6 - P4),cross(Z5,P6 - P5),cross(Z6,[0,0,0]');
Z1,Z2,Z3,Z4,Z5,Z6];

J = simplify(J)

J_inv = inv(J);
J_inv = simplify(J_inv)
end