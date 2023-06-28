%----------------------------- Sem compensador --------------------------------------------
s = tf('s')
G = 1200 / (79.2e-9 * s^2 + 0.3e-3 * s + 12) 	% FUNÇÃO TRANSFERENCIA DO CONVERSOR BUCK
rlocus(G)
%------------------------------- Compensador PD -----------------------------------------
zeta = 0.456			% RETA DE ZETA PARA ULTRAPASSAGEM PERCENTUAL DE 20%
Gpd = (s + 20107.76) 	% FUNÇÃO TRANSFERÊNCIA DO COMPENSADOR PD COM ZERO EM -20107.76, CALCULADO PARA UP% = 20% E Ts = 400us
GcompPD = G * Gpd; 		% APLICAÇÃO DO CONTROLADOR PD NO SISTEMA DO CONVERSOR BUCK
rlocus(GcompPD);		% ESTUDO DO LGR DO CONVERSOR BUCK COM CONTROLE PD
sgrid(zeta,0)
k1 = 1.06e-6;		% GANHO K PARA QUE O LGR DO SISTEMA COMPENSADO CRUZE A RETA DE ZETA  
G1 = GcompPD * k1;		% ESTUDO DA RESPOSTA EM MALHA FECHADA DO SISTEMA COM CONTROLADOR PD
T1 = feedback(G1,1);
step(T1)
%----------------------------------- Compensador PID -------------------------------------------------------------------
Gpi = (s+8500)/s		% FUNÇÃO TRANSFERÊNCIA DO CONTROLADOR PI, COM ZERO ARBITRADO EM -8500
GcompPID = G * Gpd * Gpi;	% APLICAÇÃO DO CONTROLADOR PID NO SISTEMA DO CONVERSOR BUCK
rlocus(GcompPID);		% ESTUDO DO LGR DO SISTEMA COM CONTROLADOR PID
sgrid(zeta,0)		
k2 = 1.84e-6;		% GANHO K PARA QUE O LGR CRUZE A RETA DE ZETA
G2 = GcompPID * k2;		% FUNÇÃO TRANFERÊNCIA FINAL DO SISTEMA COM CONTROLE PID, PARA ESTUDO DA RESPOSTA AO DEGRAU EM MALHA FECHADA
T2 = feedback(G2,1);
step(T2)			% RESPOSTA FINAL COM UP% = 18% E Ts = 438us