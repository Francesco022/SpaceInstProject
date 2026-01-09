close; clear; clc;



start_time = datetime(2026,3,20,0,0,0);
end_time = datetime(2026,3,21,0,0,0);
dt = 60;
sc = satelliteScenario(start_time,end_time,dt);


tle = 'transat.tle';
sat = satellite(sc, tle);

v = satelliteScenarioViewer(sc);

% Aggiungi assi del satellite


