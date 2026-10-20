# Comau: problemi aperti

## Tremore del robot sopra i ~50 mm/s (aperto il 2026-10-09)

**Sintomo.** Con il robot reale (R1C, 10.1.8.10, modalità REMOTE, pannello al 100%) un movimento
PILZ LIN sopra circa 50 mm/s trema molto. Sotto è normale. Il robot arriva a destinazione senza errori.

**Misura** (registrazione di `/joint_trajectory_controller/controller_state` a 100 Hz,
script `pdl_tools/record_controller_state.py`, movimento a 100 mm/s):
- la traiettoria comandata è liscia (velocità max 0,29–0,34 rad/s, senza oscillazioni);
- l'inseguimento è sano: ritardo costante 0,02–0,04 rad (~100 ms), errore finale 0, nessun abort;
- la velocità *misurata* tra due aggiornamenti di stato (20 ms) è molto irregolare (cv 65–75%) e ha un
  periodo di **3 campioni = 60 ms (~16 Hz)**: autocorrelazione al lag 3 = 0,87; 13–14% degli intervalli
  sotto il 25% della media. In pratica il robot fa stop-and-go a ~16 Hz.

**Prove fatte, senza effetto sul tremore.**
- `/comau/fly_lin_velocity` a 0,15, 0,3, 0,5 (prima del fix PDL qui sotto: ininfluente per costruzione),
  0,13 e 0,01 (dopo il fix: il robot si muove "come prima", anche con 0,01 m/s, che dovrebbe farlo
  strisciare => il limite sembra **non applicato** dal controller).
- `/comau/stream_decimation` a 1, 3, 4.
- `/comau/fly_dist`: non provato in modo sistematico.

**Modifiche già fatte (restano valide).**
- PDL: `vr_lin_spd` e `vr_fly_dist` erano locali e a 0,0 in `arm1_handler`; ora sono
  `EXPORTED FROM motion_server`, condivise con `arm1_handler`.
- PDL: tolta la `WRITE "Moving!"` che veniva eseguita a ogni `MOVEFLY`.

**Ipotesi, da verificare (in ordine).**
1. Il `MOVEFLY` ignora `$LIN_SPD` perché non è indicato il tipo di velocità (nelle traiettorie
   cartesiane del PDL c'è `$SPD_EMT := SPD_LIN`, nel `MOVEFLY` di `arm1_handler` manca), oppure il valore
   non arriva davvero. Test: far scrivere su file (`UD:/usr/fly_log.txt`) `vr_lin_spd` e `vr_fly_dist`
   ricevuti da `arm1_handler`; poi provare ad aggiungere `$SPD_EMT = SPD_LIN` (e/o `$SPD_OPT`) nella `WITH`.
2. Il ciclo di `arm1_handler` dura ~60 ms (non ~21 ms): `DELAY 1` e `DELAY ROUND(1/50*1000)` vengono
   probabilmente arrotondati a un'unità di tempo più grande. Test: togliere `DELAY 1` e abbassare il
   `DELAY` del ciclo, poi rimisurare con la registrazione.
3. `$FLY_DIST` (5,0, unità non nota) non produce il "volo" tra i punti: provare valori molto diversi
   (0, 1, 20, 100) con `/comau/fly_dist`.
4. Se niente funziona: cambiare approccio di streaming lato controller (es. sensor tracking, già nel
   driver come tipo di moto `'S'`) invece di una serie di `MOVEFLY ... ADVANCE`.

**Prova in corso (2026-10-13).** Caricato su `UD:/usr/comau_ros2_server` un `arm1_handler.pdl` di prova
(solo quello è cambiato, `.pdl` da compilare dal TP):
- `MOVEFLY` con `$SPD_EMT = SPD_LIN` aggiunto nella `WITH` (ipotesi 1);
- log di `vr_lin_spd`, `vr_fly_dist`, `$ARM_OVR` sullo schermo del controller (`[ARM1_SERVER ]`), solo quando
  cambiano. La prima versione scriveva anche su `UD:\USR\fly_log.txt` con `OPEN FILE ... 'rw'`, ma
  all'avvio `arm1_handler` dava "errore READ o operazioni su file 577" (probabile "file inesistente":
  `'rw'` non crea il file): tolto il log su file.
Punti marcati `DIAGNOSTIC` / `TEST (hypothesis 1)` nel sorgente, per il revert.

**Esito (2026-10-13): ipotesi 1 confermata.**
- `vr_lin_spd` (1,25) e `vr_fly_dist` (5,0) arrivano correttamente a `arm1_handler`: il problema non era il passaggio dei valori.
- Con `$SPD_EMT = SPD_LIN` nella `WITH` il limite `$LIN_SPD` ora ha effetto. Con `/comau/fly_lin_velocity`
  = 0,1 il tremore **non c'è più**, ma il controller di traiettoria dà errore perché il robot non riesce a
  seguire la traiettoria (limite troppo vicino alla velocità comandata: nessun margine per recuperare il ritardo).
- Da fare: trovare il limite con margine (prova 0,12 / 0,15 / 0,2 per un moto a 100 mm/s) e/o far calcolare
  al driver il limite in funzione della velocità cartesiana comandata. Registrare ogni prova con
  `record_controller_state.py`.
- Limite fisso non basta: con 0,2 va bene a 100 mm/s ma a 50 mm/s vibra (conta il rapporto limite/velocità).

**Velocità MOVEFLY adattiva nel driver (2026-10-13, da provare sul robot).** `comau_hw_interface.cpp` manda
con ogni `MOVEFLY` `lin_spd = clamp(margin * v_traj + gain * ritardo_TCP, fly_speed_min, fly_lin_velocity)`,
con `v_traj` dalla cinematica diretta (KDL, URDF `original_xml`, catena radice -> `flange`) dei comandi.
- Default: margin 1,3, gain 1,0 1/s, min 0,01 m/s; `fly_lin_velocity` (1,25) è ora solo il tetto.
- Topic: `/comau/fly_speed_adaptive` (Bool, `false` = comportamento vecchio), `/comau/fly_speed_margin`,
  `/comau/fly_speed_gain` (Float32); velocità effettivamente inviata su `/comau/fly_lin_velocity_sent`.
- Parametri hardware equivalenti: `fly_speed_adaptive`, `fly_speed_margin`, `fly_speed_gain`, `fly_speed_min`,
  `fk_base_link`, `fk_tip_link`.
- All'avvio il log deve mostrare `Adaptive MOVEFLY speed: forward kinematics ... (6 joints)`; altrimenti il
  driver ricade sul limite fisso.
- Plot in tempo reale di posizione/velocità della flangia (comandata, misurata, `lin_spd` inviata, errore,
  cv della velocità misurata sugli ultimi 2 s): `python3 pdl_tools/plot_ee_live.py [--csv file.csv]`.
- Da verificare: tratti di sola rotazione (velocità lineare ~0, limitata dal minimo).
- Il container `robot-planner` ricompila `/ws/robots` all'avvio con `|| true`: un errore di compilazione non
  blocca l'avvio e resta il driver vecchio. Ricompilare solo il driver (a driver fermo):
  `docker exec robot-planner bash -c "source /opt/ros/jazzy/setup.bash && source /ws/ros_overlay/install/setup.bash && colcon --log-base /ws/ros_overlay/log build --symlink-install --build-base /ws/ros_overlay/build --install-base /ws/ros_overlay/install --base-paths /ws/robots --packages-select comau_hardware_interface"`.
Da fare: compilare, rimisurare a 100 mm/s con `record_controller_state.py` e confrontare l'autocorrelazione al
lag 3 (0,87 prima).

**Prova con il driver adattivo (2026-10-13): inseguimento non buono. In sospeso.**
Moto di 100 mm in Y a ~200 mm/s, registrato con `plot_ee_live.py`:
- `lin_spd` inviata ~300 mm/s, ma la velocità misurata non supera **~150 mm/s** (piatta): ritardo fino a 44 mm.
  Non è il TP: in coda la misurata segue la `lin_spd` quasi 1:1.
- In partenza stop-and-go finché il ritardo è piccolo (bersaglio vicino); oltre ~20 mm di ritardo il moto è fluido.
- Coda lenta (~0,8 s): a traiettoria finita resta solo `gain × ritardo` (gain 1/s, costante di tempo 1 s) + minimo 10 mm/s.
- Ipotesi: tetto dovuto all'esecuzione tratto per tratto dei `MOVEFLY` corti (~12 mm ogni ~60 ms) con
  `$FLY_DIST = 5` e `$STRESS_PER = 50` (raccordo piccolo e prudente).

Prossimi passi proposti, in ordine:
1. Senza ricaricare niente: `/comau/fly_dist` a 20 e 50, `/comau/fly_speed_gain` a 4; registrare ogni prova con
   `plot_ee_live.py --csv`. Se il tetto si alza, l'ipotesi è confermata (è anche l'ipotesi 3).
2. PDL: `$STRESS_PER` da 50 a 100 (richiede caricamento sul controller).
3. Driver: anticipare il bersaglio di ~100 ms estrapolando la traiettoria (il JTC comanda solo `position`).
4. Se lo streaming di `MOVEFLY` resta il limite: sensor tracking (moto `'S'`, ipotesi 4).

**Deploy: attenzione al percorso.** `deploy_to_robot.sh` con il default `UD:\USR\comau_ros2_server` perde i
backslash nel client `ftp` e carica in `UD:USRcomau_ros2_server` (cartella sbagliata, esiste ancora con i 5
`.pdl` del 2026-10-13 09:58). Usare `-t UD:/usr/comau_ros2_server`. Lo script stampa `DEPLOYMENT COMPLETE`
anche se il login fallisce: controllare i `226 Transfer complete`.

**Stato attuale.**
- Controller (`UD:/usr/comau_ros2_server`): i 5 `.pdl` del 2026-10-13 (`arm1_handler` con `$SPD_EMT = SPD_LIN`
  e log a schermo `DIAGNOSTIC`), compilati dal TP. Presenti anche file di prova: `set_ovr`, `ovr_log`, `ovr_log2`,
  `ovr_log3`, `test_arm_ovr` (innocui).
- Driver ROS (`comau_hardware_interface`): comandi bloccati finché non c'è Drive ON + START; override
  inviato al controller (default 100%) con comando `'V'`; topic di regolazione `/comau/speed_override`,
  `/comau/fly_lin_velocity`, `/comau/fly_dist`, `/comau/stream_decimation`.
- I valori impostati sui topic valgono solo fino al riavvio di ROS. **Se `fly_lin_velocity` è rimasto a
  0,01, i movimenti andranno lentissimi**: rimetterlo a 1,25 (valore predefinito).
- Il pannello (TP) deve restare al **100%**: fa da moltiplicatore finale (a 1% il robot va all'1%).
  ROS non legge ancora la velocità del pannello; se è sotto 100% il robot resta indietro, il controller di
  traiettoria abortisce dopo 2 s e il robot "rimbalza" (la coda del controller esegue prima l'arrivo e
  poi il punto di hold).

**Prove del 2026-10-13 sera (driver in esecuzione, valori via topic/parametri).** CSV in `pdl_tools/prove/`.
- `fly_dist` 20 + `fly_speed_gain` 4 (`fly20_gain4.csv`): tetto di 150 mm/s superato, il robot segue fino a ~330 mm/s, coda
  ~0,2 s (era 0,8). Lo scarto non è più un tetto di velocità ma un **ritardo costante ~0,15 s** (scarto/velocità uguale a
  tutte le velocità). Interruzioni a ~350 mm/s: tolleranza `trajectory` 0,2 rad del JTC superata di poco (giunto 2).
- `fly_dist` 50 (`fly50_gain4.csv`): ritardo invariato (0,153 s), ok fino a ~420 mm/s; velocità in crociera un po' meno
  regolare (cv ~20% contro ~16%).
- `fly_dist` 50 + tolleranza JTC `constraints.joint_N.trajectory` 0,4 rad via `ros2 param set` (letta a runtime)
  (`fly50_tol04.csv`): 20 moti da 120 a 720 mm/s tutti ok, ritardo 0,12–0,18 s, coda ~0,2 s. A ~700 mm/s un moto su tre
  interrotto (giunto 2 a 0,401 rad): partenza lenta (~250 ms quasi fermo) e `lin_spd` al tetto di 1,25 m/s.
- Dopo un'interruzione il robot va avanti ancora ~5 cm (`MOVEFLY` in coda) e poi torna indietro lentamente (~110 mm/s).
- **Anticipo del bersaglio (prossimo passo 3) scartato**: estrapolato dal solo comando, il bersaglio supera il punto finale di
  2–6 cm per ~140 ms prima di ogni arrivo (simulazione con 2–4 m/s²), perché il driver non sa quando inizia la frenata.
  Si potrebbe fare solo con la traiettoria intera (controller su misura).
- Codice: `fly_dist` predefinito 50 e `fly_speed_gain` predefinito 4 (compilato e riavviato in `mirror` la sera del 2026-10-13).
- Tolleranza: generata uguale per tutti i robot in `robot-planner/ros-package-creator/routes_manipulator_export.py`
  (`"trajectory": 0.2`); il valore 0,4 vale solo fino al riavvio di ROS.
- Da fare per accorciare il ritardo: PDL `motion_server` scrive `vs_motion_type:` sullo schermo a ogni messaggio (100 Hz);
  ciclo di `arm1_handler` ~60 ms (ipotesi 2).

**Fatto la sera del 2026-10-13 (da verificare sul robot).**
- Tolleranza JTC `trajectory` 0,4 rad permanente: nel pacchetto `PRJ043/manipulator/rospackage_8f007f0f/config/ros2_controllers_real.yaml`
  e nel generatore (`routes_manipulator_export.py`, solo variante real = Comau; sim resta 0,2).
- Driver: su un goal del JTC interrotto (`<stop_cancel_action>/_action/status`, STATUS_ABORTED di un goal visto in esecuzione)
  annulla la coda del controller come per uno stop: il robot dovrebbe fermarsi senza "rimbalzo" e ripartire al goal successivo.
- Da provare: 2–3 moti a 500–600 mm/s (devono passare); uno a ~800 mm/s per vedere l'interruzione senza rimbalzo.

**Jerk (richiesta dell'utente, da decidere).** PILZ fa profili trapezoidali (gradino di accelerazione), nessun parametro di jerk;
in `pilz_cartesian_limits.yaml` la frenata (-5,0) è più del doppio dell'accelerazione (2,25). Proposta: A) subito
`max_trans_dec: -2.25`; C) ricalcolo dei tempi nel `planning_server` con profilo a S lungo il percorso PILZ (limite di jerk in
m/s³, percorso LIN invariato), da provare prima in simulazione. Alternativa B: `AddRuckigTrajectorySmoothing` di MoveIt.
