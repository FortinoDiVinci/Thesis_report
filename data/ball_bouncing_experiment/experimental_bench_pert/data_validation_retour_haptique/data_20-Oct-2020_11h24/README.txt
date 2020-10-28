Le calcul de la commande de vitesse a été corrigé. 
Le calcul du PID était fait avec l'approximation que la période d'échantillonnage était constante.
Or, lors du passage du contrôle en couple au contrôle en vitesse, le temps était de 30ms et non 1ms.

L'action intégrale est maintenant calculée correctement, sans considérer l'échantillon temporel constant:

Ancien calcul :

limitReached ? error_sum = 0 : error_sum += error; // anti windup
cmd = Kp * error + Ki * error_sum * (t - t_old).toSec();

Nouveau calcul :

limitReached ? error_sum += 0 : error_sum += error * (t - t_old).toSec(); // anti windup
cmd = Kp * error + Ki * error_sum;