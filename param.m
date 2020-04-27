%Parametri
opacita = 0.4;

g=0;
l = 0.1; %lato oggetto

l0 = 0.5;
l1z = 0.5;
l1y = 0.8;
l2 = 0.3;
l3 = 0.4;
g1z = l1z/2;
g1y = l1y/2;
g2 = l2/2;
g3 = l3/2;
L = [l0 l1z l1y l2 l3];
Le = [l0;l1z;l1y;l2;l3;g1z;g1y;g2;g3];

m_gripper = 0;
m_oggetto = 0;

m0  = 0; %non serve
mz1 = 0;
my1 = 0;
m2 = 0;
m3 = 0;

%Inerzie approssimate per metodo tre masse, calcolate in base al valore
%della massa
Jg0x = 0; %non serve
Jg0y = 0; %non serve
Jg0z = 0; %non serve

Jgz1x = 0;
Jgz1y = 0;
Jgz1z = 0;

Jgy1x = 0;
Jgy1y = 0;
Jgy1z = 0;

Jg2x = 0;
Jg2y = 0;
Jg2z = 0;

Jg3x = 0;
Jg3y = 0;
Jg3z = 0;

J_oggetto = 0.0;
J_gripper = 0.0;
J_end_effector = 0;

Vmax = [2 2 2]'; %rad/s
Amax = [5 5 5]'; %rad/s^2
Dmax = [5 5 5]'; %rad/s^2