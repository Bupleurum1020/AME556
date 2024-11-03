%% Plot3
clear
clc

O = [0;0;0];  % origin
A = [1;1;2];  % point A
B = [2;3;4];  % point B

OA = [O,A];  % OA Vector
OB = [O,B];  % OB Vector
AB = [A,B];  % AB Vector


plot3(OA(1,:), OA(2,:), OA(3,:), 'red')
grid on
axis equal

hold on
plot3(OB(1,:), OB(2,:), OB(3,:), 'blue')

hold on
plot3(AB(1,:), AB(2,:), AB(3,:), 'yellow')

%% Patch
clear
clc

O = [0;0;0];  % origin
A = [1;1;2];  % point A
B = [1;3;2];  % point B

tri = [O,A,B]; % triangle OAB

patch(tri(1,:), tri(2,:), tri(3,:), 'blue');
view(3)
axis equal

%% Symbolic scalar variable
clear
clc

syms a b c % symbolic scalar variable

A = [a , a^2 , b;
     b , b/2 , c;
     c , 2*c , a]; % symbolic matrix A

B = inv(A); % inverse of matrix A

C = A*B;
C_simple = simplify(C);

% A_value = subs(A, a, 2);
A_value = subs(A, [a,b,c], [2,1,2]);
