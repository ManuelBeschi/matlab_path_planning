clear all;close all;clc;


% inizializza ROS (se non lo è già')
try
    rosinit
end

% crea un'instanza di collision checking. Richede che ci sia MoveIt attivo
checker=CollisionChecker;
% inizializza il nome del gruppo (insieme di link) da usare
checker.init('cembre',true);



home=[-1.0000   -3.1414    2.5708   -1.0000    1.5708   -0.0001   -0.000]';
down=[1.5707   -1.5571    1.5708   -1.5708    1.5707   -0.0001    0.0001]';
p1=[-0.5000   -3.1414    2.5708   -1.0000    1.5708   -0.0001   -0.000]';


% self explanatory
position=checker.getActualPosition();

% uno se è ok,  0 se in collisione
checker.check(position)

% il path è un insieme di posizioni in colonna
s=linspace(0,1,21)';
for idx=1:length(s)
    path(:,idx)=home*(1-s(idx))+p1*s(idx);
end

% uno se è ok,  0 se è in collisione almeno una posizione
if (~checker.checkPath(path))
    disp('path is in collision');
end

