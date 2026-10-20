#!/usr/bin/env python3
"""Plot in tempo reale di posizione e velocità della navetta (OPC UA) e della flangia del Comau sull'asse Y (ROS).

  - Robot (ROS): /joint_trajectory_controller/controller_state -> cinematica diretta (PyKDL, URDF da
    /robot_description o da file) -> Y della flangia [mm] e velocità lungo Y [mm/s], con segno.
    Misurata (continua) e comandata (tratteggiata), come in plot_ee_live.py.
  - Navetta (OPC UA, PLC Siemens): Stato/posizione_lineare [mm] e Stato/velocita_attuale [mm/s] di DB_OPC_Navetta,
    letti a polling con una sola sessione (nessuna sottoscrizione: il PLC ne accetta poche).

Due grafici con lo stesso asse tempo: in alto le posizioni, in basso le velocità. Le due sorgenti sono
datate con l'orologio di questo PC alla ricezione, quindi tra loro sono allineate entro qualche decina di ms
(ritardo di rete e periodo di polling), non meglio.

Le quote sono nei rispettivi sistemi di riferimento: la navetta è la quota lineare del PLC (0 .. 509.5 mm), la
flangia è la Y nel frame radice dell'URDF. Per confrontare gli spostamenti:
  --relative            toglie a entrambe la prima quota ricevuta
  --shuttle-sign -1     inverte il verso della navetta (nel gemello digitale il giunto è invertito rispetto a Y)
  --shuttle-offset MM   somma un offset alla quota della navetta (dopo il segno)

Uso (sull'host; il planner può girare anche sulla Jetson):
  source /opt/ros/jazzy/setup.bash
  python3 plot_shuttle_flange_y.py --planner-host 10.1.8.12
  python3 plot_shuttle_flange_y.py --planner-host 10.1.8.12 --relative --shuttle-sign -1 --csv prova.csv
  python3 plot_shuttle_flange_y.py --mock          # dati simulati, senza ROS né PLC

Pausa: pulsante in basso a destra o barra spaziatrice. In pausa il grafico è fermo e mostra tutto lo
storico in memoria: zoom con la lente della barra o con la rotella (tempo), Shift+rotella (asse Y).
"""
import argparse
import asyncio
import collections
import csv
import math
import os
import sys
import threading
import time

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button

PLC_URL = 'opc.tcp://10.1.8.1:4840'
PLC_NAMESPACE = 'http://CIBUS2026'
# ID numerici dallo snapshot cibus_plc_nodeset.json (cambiano se il programma del PLC viene ricaricato:
# in quel caso il nodo viene cercato per nome sotto ServerInterfaces/CIBUS2026/DB_OPC_Navetta/Stato)
SHUTTLE_NODES = {
    'pos': ('posizione_lineare', 1470),
    'vel': ('velocita_attuale', 1471),
}


class WindowVelocity:
    """Velocità con segno su una finestra fissa: confronta il campione con quello più vecchio di `dt` secondi.

    Con dt=0 usa il campione precedente. Non scarta i campioni uguali, altrimenti gli stop veri sparirebbero."""

    def __init__(self, dt):
        self.dt = dt
        self.hist = collections.deque(maxlen=400)

    def update(self, t, y):
        self.hist.append((t, y))
        for t_old, y_old in reversed(self.hist):
            if t - t_old <= 1e-6:
                continue
            if t_old <= t - self.dt + 0.002:
                return (y - y_old) / (t - t_old)
        return float('nan')


class Hub:
    """Contenitore dei dati condiviso tra i thread delle sorgenti e il grafico."""

    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.t0 = None
        n_ros = int(args.history * 250) + 100
        n_plc = int(args.history * max(args.poll_hz, 1.0) * 2) + 100
        self.flange = collections.deque(maxlen=n_ros)       # (t, y mm, vy mm/s) misurata
        self.flange_ref = collections.deque(maxlen=n_ros)   # (t, y mm, vy mm/s) comandata
        self.shuttle = collections.deque(maxlen=n_plc)      # (t, pos mm, vel mm/s)
        self.status = {'ros': 'in attesa del robot', 'opcua': 'connessione al PLC...'}
        self.last_rx = {'ros': None, 'opcua': None}         # istante (monotonic) dell'ultimo dato
        self.first = {}                                     # primo valore di ogni serie (per --relative)
        self.csv_file = None
        self.csv_writer = None
        if args.csv:
            self.csv_file = open(args.csv, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['t', 'serie', 'pos_mm', 'vel_mm_s'])

    def _rel(self, t_mono):
        if self.t0 is None:
            self.t0 = t_mono
        return t_mono - self.t0

    def _offset(self, key, value):
        """Con --relative la prima quota ricevuta di una serie è lo zero di quella serie."""
        if not self.args.relative or not math.isfinite(value):
            return 0.0
        return self.first.setdefault(key, value)

    def _log(self, t, name, pos, vel):
        if self.csv_writer:
            self.csv_writer.writerow(['%.4f' % t, name, '%.3f' % pos, '%.3f' % vel])

    def add_flange(self, t_mono, y, vy, y_ref, vy_ref):
        with self.lock:
            t = self._rel(t_mono)
            zero = self._offset('flange', y)    # stesso zero per misurata e comandata
            y, y_ref = y - zero, y_ref - zero
            self.flange.append((t, y, vy))
            self.flange_ref.append((t, y_ref, vy_ref))
            self.last_rx['ros'] = time.monotonic()
            self._log(t, 'flangia', y, vy)
            self._log(t, 'flangia_comandata', y_ref, vy_ref)

    def add_shuttle(self, t_mono, pos, vel):
        a = self.args
        with self.lock:
            t = self._rel(t_mono)
            pos = a.shuttle_sign * pos + a.shuttle_offset
            pos -= self._offset('shuttle', pos)
            vel = a.shuttle_sign * vel
            self.shuttle.append((t, pos, vel))
            self.last_rx['opcua'] = time.monotonic()
            self._log(t, 'navetta', pos, vel)

    def set_status(self, source, text):
        with self.lock:
            self.status[source] = text

    def snapshot(self):
        with self.lock:
            return (np.array(self.flange) if self.flange else np.empty((0, 3)),
                    np.array(self.flange_ref) if self.flange_ref else np.empty((0, 3)),
                    np.array(self.shuttle) if self.shuttle else np.empty((0, 3)),
                    dict(self.status), dict(self.last_rx))

    def close(self):
        if self.csv_file:
            self.csv_file.close()


# --------------------------------------------------------------------------------------------------
# Robot: ROS 2
# --------------------------------------------------------------------------------------------------
def start_ros(hub, args):
    """Avvia il nodo ROS in un thread. Restituisce la funzione di chiusura."""
    # Stessa configurazione DDS del container robot-planner (va impostata prima di importare rclpy).
    # Il file cyclonedds.xml elenca i peer da variabili d'ambiente: PLANNER_HOST è la Jetson.
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    if args.planner_host:
        os.environ['PLANNER_HOST'] = args.planner_host
    dds_xml = os.path.expanduser('~/ws/commons/cyclonedds.xml')
    if os.path.exists(dds_xml):
        os.environ.setdefault('CYCLONEDDS_URI', dds_xml)

    try:
        import PyKDL as kdl
        import rclpy
        from control_msgs.msg import JointTrajectoryControllerState
        from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from urdf_parser_py.urdf import URDF
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from plot_ee_live import chain_from_urdf
    except ImportError as exc:
        sys.exit('Moduli ROS non trovati (%s). Esegui prima: source /opt/ros/jazzy/setup.bash' % exc)

    class FlangeYNode(Node):
        def __init__(self):
            super().__init__('shuttle_flange_y_plot')
            self.fk = None
            self.chain = None       # il solver tiene solo un riferimento C++ alla catena: va tenuta viva
            self.q = None
            self.movable = []
            self.warned = False
            self.v_meas = WindowVelocity(args.meas_dt)
            self.v_ref = WindowVelocity(0.0)
            if args.urdf:
                with open(args.urdf) as f:
                    self.set_urdf(f.read())
            else:
                qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self.create_subscription(String, '/robot_description', lambda m: self.set_urdf(m.data), qos)
                hub.set_status('ros', 'in attesa dell\'URDF su /robot_description')
            self.create_subscription(JointTrajectoryControllerState, args.topic, self.on_state,
                                     QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))

        def set_urdf(self, xml):
            if self.fk is not None:
                return
            robot = URDF.from_xml_string(xml)
            self.chain, self.movable = chain_from_urdf(robot, args.base, args.tip)
            self.fk = kdl.ChainFkSolverPos_recursive(self.chain)
            self.q = kdl.JntArray(self.chain.getNrOfJoints())
            self.get_logger().info('Cinematica: %s -> %s, giunti %s'
                                   % (args.base or robot.get_root(), args.tip, self.movable))
            hub.set_status('ros', 'in attesa di %s' % args.topic)

        def flange_y(self, names, positions):
            index = {n: i for i, n in enumerate(names)}
            for k, name in enumerate(self.movable):
                if name not in index or index[name] >= len(positions):
                    return None
                self.q[k] = positions[index[name]]
            frame = kdl.Frame()
            self.fk.JntToCart(self.q, frame)
            return frame.p.y() * 1000.0

        def on_state(self, msg):
            if self.fk is None:
                return
            t_rx = time.monotonic()
            t_msg = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if t_msg == 0.0:
                t_msg = self.get_clock().now().nanoseconds * 1e-9
            y_ref = self.flange_y(msg.joint_names, msg.reference.positions)
            y_meas = self.flange_y(msg.joint_names, msg.feedback.positions)
            if y_ref is None or y_meas is None:
                if not self.warned:
                    self.get_logger().warn('Giunti %s non trovati in %s' % (self.movable, list(msg.joint_names)))
                    self.warned = True
                return
            # Le velocità usano i timestamp del messaggio (precisi); l'asse tempo del grafico usa l'orologio
            # locale alla ricezione, comune alla navetta (gli orologi del PC e della Jetson non sono sincroni).
            hub.add_flange(t_rx, y_meas, self.v_meas.update(t_msg, y_meas), y_ref, self.v_ref.update(t_msg, y_ref))
            hub.set_status('ros', 'ok')

    rclpy.init()
    node = FlangeYNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass    # chiusura normale: rclpy.try_shutdown() in close()

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    def close():
        # Prima si chiude il contesto (lo spin termina), poi si aspetta il thread: lasciarlo girare
        # mentre l'interprete esce manda in crash il processo (std::terminate)
        rclpy.try_shutdown()
        spin_thread.join(timeout=2.0)
        executor.shutdown()
        try:
            node.destroy_node()
        except Exception:
            pass
    return close


# --------------------------------------------------------------------------------------------------
# Navetta: OPC UA
# --------------------------------------------------------------------------------------------------
async def resolve_nodes(client, ua, args):
    """Nodi (posizione, velocità) della navetta: da riga di comando, dall'ID noto o cercati per nome."""
    idx = await client.get_namespace_index(args.opcua_namespace)
    overrides = {'pos': args.pos_node, 'vel': args.vel_node}
    nodes = {}
    for key, (name, fallback_id) in SHUTTLE_NODES.items():
        if overrides[key]:
            nodes[key] = client.get_node(overrides[key])
            continue
        node = client.get_node(ua.NodeId(fallback_id, idx))
        try:
            ok = (await node.read_browse_name()).Name == name
        except Exception:
            ok = False
        if not ok:
            root = client.get_node(ua.NodeId('ServerInterfaces', 3))
            path = ['%d:%s' % (idx, p) for p in ('CIBUS2026', 'DB_OPC_Navetta', 'Stato', name)]
            node = await root.get_child(path)
        nodes[key] = node
    return nodes['pos'], nodes['vel']


async def opcua_loop(hub, args, stop):
    from asyncua import Client, ua
    period = 1.0 / args.poll_hz
    v_derived = WindowVelocity(args.meas_dt_shuttle) if args.shuttle_vel_from_pos else None
    while not stop.is_set():
        try:
            hub.set_status('opcua', 'connessione a %s ...' % args.opcua_url)
            async with Client(args.opcua_url, timeout=4) as client:
                node_pos, node_vel = await resolve_nodes(client, ua, args)
                hub.set_status('opcua', 'ok')
                while not stop.is_set():
                    t_a = time.monotonic()
                    pos, vel = await client.read_values([node_pos, node_vel])
                    t_b = time.monotonic()
                    if isinstance(pos, (int, float)) and isinstance(vel, (int, float)):
                        t = 0.5 * (t_a + t_b)       # il valore è di un istante a metà tra richiesta e risposta
                        if v_derived is not None:
                            vel = v_derived.update(t, float(pos))
                            if math.isnan(vel):
                                vel = 0.0
                        hub.add_shuttle(t, float(pos), float(vel))
                    await asyncio.sleep(max(0.0, period - (time.monotonic() - t_a)))
        except Exception as exc:
            hub.set_status('opcua', 'errore: %r (riprovo)' % (exc,))
            await asyncio.sleep(3.0)


def start_opcua(hub, args):
    stop = threading.Event()
    threading.Thread(target=lambda: asyncio.run(opcua_loop(hub, args, stop)), daemon=True).start()
    return stop.set


# --------------------------------------------------------------------------------------------------
# Dati simulati (senza ROS né PLC)
# --------------------------------------------------------------------------------------------------
def start_mock(hub, args):
    stop = threading.Event()
    rng = np.random.default_rng(1)

    def shuttle_pos(t):
        # va e vieni 20 <-> 480 mm a 50 mm/s con 2 s di sosta agli estremi
        stroke, speed, pause = 460.0, 50.0, 2.0
        leg = stroke / speed
        phase = t % (2 * (leg + pause))
        if phase < leg:
            return 20.0 + speed * phase
        if phase < leg + pause:
            return 480.0
        if phase < 2 * leg + pause:
            return 480.0 - speed * (phase - leg - pause)
        return 20.0

    def run():
        v_flange = WindowVelocity(args.meas_dt)
        v_ref = WindowVelocity(0.0)
        t0 = time.monotonic()
        next_plc = 0.0
        while not stop.is_set():
            t = time.monotonic() - t0
            # robot: segue la navetta con 0,4 s di ritardo e un po' di rumore, a 100 Hz
            y_ref = 0.9 * shuttle_pos(t - 0.1) - 200.0
            y = 0.9 * shuttle_pos(t - 0.4) - 200.0 + rng.normal(0.0, 0.05)
            hub.add_flange(time.monotonic(), y, v_flange.update(t, y), y_ref, v_ref.update(t, y_ref))
            hub.set_status('ros', 'ok (simulato)')
            if t >= next_plc:
                next_plc = t + 1.0 / args.poll_hz
                p = shuttle_pos(t)
                pv = (shuttle_pos(t + 0.01) - shuttle_pos(t - 0.01)) / 0.02
                hub.add_shuttle(time.monotonic(), p + rng.normal(0.0, 0.02), pv)
                hub.set_status('opcua', 'ok (simulato)')
            time.sleep(0.01)

    threading.Thread(target=run, daemon=True).start()
    return stop.set


# --------------------------------------------------------------------------------------------------
# Grafico
# --------------------------------------------------------------------------------------------------
def build_plot(hub, args):
    """Crea la figura e restituisce (fig, update) per FuncAnimation."""
    fig, (ax_pos, ax_vel) = plt.subplots(2, 1, sharex=True, figsize=(11, 7))
    axes = (ax_pos, ax_vel)
    fig.canvas.manager.set_window_title('Navetta (OPC UA) e flangia Comau su Y (ROS)')
    c_shuttle, c_flange = 'C0', 'C3'

    p_shuttle, = ax_pos.plot([], [], '-', color=c_shuttle, lw=1.5, label='navetta (OPC UA)')
    p_flange, = ax_pos.plot([], [], '-', color=c_flange, lw=1.5, label='flangia Y misurata')
    p_flange_ref, = ax_pos.plot([], [], '--', color=c_flange, lw=1, alpha=0.8, label='flangia Y comandata')
    ax_pos.set_ylabel('posizione [mm]')
    ax_pos.legend(loc='upper left', fontsize=8, framealpha=0.8)
    ax_pos.grid(True, alpha=0.3)

    v_shuttle, = ax_vel.plot([], [], '-', color=c_shuttle, lw=1.5, label='navetta (OPC UA)')
    v_flange, = ax_vel.plot([], [], '-', color=c_flange, lw=1.2, label='flangia vY misurata')
    v_flange_ref, = ax_vel.plot([], [], '--', color=c_flange, lw=1, alpha=0.8, label='flangia vY comandata')
    ax_vel.set_ylabel('velocità [mm/s]')
    ax_vel.set_xlabel('tempo [s]')
    ax_vel.legend(loc='upper left', fontsize=8, framealpha=0.8)
    ax_vel.grid(True, alpha=0.3)

    title = fig.suptitle('In attesa di dati ...')
    state = {'paused': False, 'status': ''}

    def set_lines(flange, flange_ref, shuttle):
        p_shuttle.set_data(shuttle[:, 0], shuttle[:, 1])
        v_shuttle.set_data(shuttle[:, 0], shuttle[:, 2])
        p_flange.set_data(flange[:, 0], flange[:, 1])
        v_flange.set_data(flange[:, 0], flange[:, 2])
        p_flange_ref.set_data(flange_ref[:, 0], flange_ref[:, 1])
        v_flange_ref.set_data(flange_ref[:, 0], flange_ref[:, 2])

    def describe(flange, shuttle, status, last_rx):
        now = time.monotonic()
        parts = []
        for src, label, data in (('opcua', 'navetta', shuttle), ('ros', 'flangia Y', flange)):
            if len(data):
                parts.append('%s %.1f mm  v %.1f mm/s' % (label, data[-1, 1], data[-1, 2]))
            if status[src] != 'ok' and not status[src].startswith('ok'):
                parts.append('[%s: %s]' % (label, status[src]))
            elif last_rx[src] is not None and now - last_rx[src] > 2.0:
                parts.append('[%s: nessun dato da %.0f s]' % (label, now - last_rx[src]))
        return '   |   '.join(parts) if parts else 'In attesa di dati ...'

    def update(_):
        if state['paused']:
            # In pausa non si tocca niente: lo zoom e lo spostamento restano come li lascia l'utente
            return
        flange, flange_ref, shuttle, status, last_rx = hub.snapshot()
        text = describe(flange, shuttle, status, last_rx)
        state['status'] = text
        title.set_text(text)
        series = [d for d in (flange, shuttle) if len(d)]
        if not series:
            return
        t_end = max(d[-1, 0] for d in series)
        t_first = min(d[0, 0] for d in series)
        t_start = max(t_end - args.window, t_first)
        # Solo la finestra visibile, così l'autoscala non tiene conto dei dati vecchi
        set_lines(flange[flange[:, 0] >= t_start], flange_ref[flange_ref[:, 0] >= t_start],
                  shuttle[shuttle[:, 0] >= t_start])
        ax_pos.set_xlim(t_start, max(t_end, t_start + 1e-3))
        for ax in axes:
            ax.relim()
            ax.autoscale_view(scalex=False)

    def toggle_pause(_event=None):
        state['paused'] = not state['paused']
        toolbar = getattr(fig.canvas, 'toolbar', None)
        if state['paused']:
            # Tutto lo storico in memoria (più della finestra), per poter scorrere anche indietro
            flange, flange_ref, shuttle, _status, _rx = hub.snapshot()
            set_lines(flange, flange_ref, shuttle)
            for ax in axes:
                ax.relim()
                ax.autoscale_view()
            button.label.set_text('Riprendi (spazio)')
            title.set_text('IN PAUSA   %s\nzoom: lente della barra o rotella (asse tempo), Shift+rotella (asse Y); '
                           'Home torna alla vista della pausa' % state['status'])
            if toolbar is not None:
                toolbar.update()        # svuota la cronologia delle viste
                toolbar.push_current()  # Home = vista al momento della pausa
        else:
            button.label.set_text('Pausa (spazio)')
            if toolbar is not None and getattr(toolbar, 'mode', ''):
                # esce dalla modalità zoom/pan, altrimenti i clic continuano a zoomare il grafico che scorre
                if 'zoom' in str(toolbar.mode):
                    toolbar.zoom()
                elif 'pan' in str(toolbar.mode):
                    toolbar.pan()
        fig.canvas.draw_idle()

    def on_key(event):
        if event.key == ' ':
            toggle_pause()

    def on_scroll(event):
        # Rotella: zoom sul tempo attorno al cursore; Shift+rotella: zoom sull'asse Y del grafico sotto il cursore
        if not state['paused'] or event.inaxes not in axes or event.xdata is None:
            return
        factor = 0.8 if event.button == 'up' else 1.25
        ax = event.inaxes
        if event.key == 'shift':
            lo, hi = ax.get_ylim()
            c = event.ydata
            ax.set_ylim(c - (c - lo) * factor, c + (hi - c) * factor)
        else:
            lo, hi = ax.get_xlim()
            c = event.xdata
            ax.set_xlim(c - (c - lo) * factor, c + (hi - c) * factor)
        fig.canvas.draw_idle()

    fig.tight_layout(rect=(0, 0.04, 1, 0.96))
    button = Button(fig.add_axes([0.84, 0.006, 0.14, 0.03]), 'Pausa (spazio)')
    button.on_clicked(toggle_pause)
    fig.canvas.mpl_connect('key_press_event', on_key)
    fig.canvas.mpl_connect('scroll_event', on_scroll)
    # Riferimenti da tenere vivi (il pulsante smette di funzionare se viene raccolto dal garbage collector)
    fig._pause_button = button
    fig._toggle_pause = toggle_pause
    return fig, update


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = parser.add_argument_group('robot (ROS)')
    g.add_argument('--urdf', help='file URDF (default: topic /robot_description)')
    g.add_argument('--base', default='', help='link di base (default: radice dell\'URDF)')
    g.add_argument('--tip', default='flange', help='link dell\'end effector (default: flange)')
    g.add_argument('--topic', default='/joint_trajectory_controller/controller_state')
    g.add_argument('--planner-host', default=os.environ.get('PLANNER_HOST'),
                   help='IP della macchina con il planner, usato come peer DDS (default: $PLANNER_HOST; '
                        'sulla Jetson 10.1.8.12)')
    g.add_argument('--meas-dt', type=float, default=0.05,
                   help='finestra [s] per la velocità misurata della flangia (default 0.05; 0.02 è il periodo '
                        'dello stato ma amplifica il jitter di arrivo dei campioni)')
    g = parser.add_argument_group('navetta (OPC UA)')
    g.add_argument('--opcua-url', default=PLC_URL)
    g.add_argument('--opcua-namespace', default=PLC_NAMESPACE)
    g.add_argument('--pos-node', help='NodeId della posizione (default: DB_OPC_Navetta/Stato/posizione_lineare)')
    g.add_argument('--vel-node', help='NodeId della velocità (default: DB_OPC_Navetta/Stato/velocita_attuale)')
    g.add_argument('--poll-hz', type=float, default=20.0, help='letture al secondo del PLC (default 20)')
    g.add_argument('--shuttle-vel-from-pos', action='store_true',
                   help='velocità della navetta derivata dalla posizione invece di velocita_attuale')
    g.add_argument('--meas-dt-shuttle', type=float, default=0.1,
                   help='finestra [s] della velocità derivata dalla posizione (default 0.1)')
    g.add_argument('--shuttle-sign', type=int, choices=(1, -1), default=1,
                   help='-1 inverte il verso della navetta (posizione e velocità)')
    g.add_argument('--shuttle-offset', type=float, default=0.0, help='offset [mm] sulla quota della navetta')
    g = parser.add_argument_group('grafico')
    g.add_argument('--relative', action='store_true', help='toglie a navetta e flangia la prima quota ricevuta')
    g.add_argument('--window', type=float, default=10.0, help='secondi mostrati (default 10)')
    g.add_argument('--history', type=float, default=300.0, help='secondi di storico in memoria (default 300)')
    g.add_argument('--fps', type=float, default=15.0, help='aggiornamenti del grafico al secondo')
    g.add_argument('--csv', help='salva anche i dati in questo file CSV (formato lungo: t, serie, pos, vel)')
    g = parser.add_argument_group('sorgenti')
    g.add_argument('--no-ros', action='store_true', help='non leggere il robot')
    g.add_argument('--no-opcua', action='store_true', help='non leggere la navetta')
    g.add_argument('--mock', action='store_true', help='dati simulati, senza ROS né PLC')
    return parser.parse_args(argv)


def main():
    args = parse_args()
    hub = Hub(args)
    closers = []
    if args.mock:
        closers.append(start_mock(hub, args))
    else:
        if not args.no_opcua:
            closers.append(start_opcua(hub, args))
        else:
            hub.set_status('opcua', 'disattivato (--no-opcua)')
        if not args.no_ros:
            closers.append(start_ros(hub, args))
        else:
            hub.set_status('ros', 'disattivato (--no-ros)')

    fig, update = build_plot(hub, args)
    anim = FuncAnimation(fig, update, interval=1000.0 / args.fps, cache_frame_data=False)  # noqa: F841
    try:
        plt.show()
    finally:
        for close in closers:
            close()
        hub.close()


if __name__ == '__main__':
    sys.exit(main())
