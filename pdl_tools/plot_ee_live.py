#!/usr/bin/env python3
"""Plot in tempo reale di posizione e velocità dell'end effector (flangia) del Comau.

Legge /joint_trajectory_controller/controller_state (giunti comandati e misurati), calcola la
posizione della flangia con la cinematica diretta (PyKDL, URDF da /robot_description o da file)
e mostra, per comandato (tratteggiato) e misurato (continuo):
  - posizione X, Y, Z [mm]
  - velocità del TCP [mm/s], con la velocità MOVEFLY inviata dal driver (/comau/fly_lin_velocity_sent)
  - errore di inseguimento |comandato - misurato| [mm]
Nel titolo: velocità media e coefficiente di variazione (cv) della velocità misurata negli ultimi 2 s,
la stessa misura usata per il tremore (cv 65-75% con lo stop-and-go).

Uso (sull'host, con il planner avviato nel container robot-planner):
  python3 plot_ee_live.py                          # URDF da /robot_description
  python3 plot_ee_live.py --urdf robot.urdf        # URDF da file
  python3 plot_ee_live.py --window 5 --csv prova_50mm.csv

Pausa: pulsante in basso a destra o barra spaziatrice. In pausa il grafico è fermo e mostra tutto lo
storico in memoria: zoom con la lente della barra o con la rotella (tempo), Shift+rotella (asse Y).
"""
import argparse
import collections
import csv
import os
import sys
import threading

# Stessa configurazione DDS del container robot-planner (va impostata prima di importare rclpy)
_DDS_XML = os.path.expanduser('~/ws/commons/cyclonedds.xml')
os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
if os.path.exists(_DDS_XML):
    os.environ.setdefault('CYCLONEDDS_URI', _DDS_XML)

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import PyKDL as kdl  # noqa: E402
import rclpy  # noqa: E402
from control_msgs.msg import JointTrajectoryControllerState  # noqa: E402
from matplotlib.animation import FuncAnimation  # noqa: E402
from matplotlib.widgets import Button  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Float32, String  # noqa: E402
from urdf_parser_py.urdf import URDF  # noqa: E402


def chain_from_urdf(robot, base, tip):
    """Catena KDL base -> tip, costruita come kdl_parser. Restituisce (chain, nomi dei giunti mobili)."""
    root = base or robot.get_root()
    chain = kdl.Chain()
    movable = []
    for name in robot.get_chain(root, tip, joints=True, links=False, fixed=True):
        j = robot.joint_map[name]
        xyz = j.origin.xyz if j.origin is not None and j.origin.xyz is not None else [0.0, 0.0, 0.0]
        rpy = j.origin.rpy if j.origin is not None and j.origin.rpy is not None else [0.0, 0.0, 0.0]
        frame = kdl.Frame(kdl.Rotation.RPY(*rpy), kdl.Vector(*xyz))
        axis = kdl.Vector(*(j.axis if j.axis is not None else [1.0, 0.0, 0.0]))
        if j.type in ('revolute', 'continuous'):
            joint = kdl.Joint(name, frame.p, frame.M * axis, kdl.Joint.RotAxis)
            movable.append(name)
        elif j.type == 'prismatic':
            joint = kdl.Joint(name, frame.p, frame.M * axis, kdl.Joint.TransAxis)
            movable.append(name)
        else:
            joint = kdl.Joint(name, kdl.Joint.Fixed)
        chain.addSegment(kdl.Segment(j.child, joint, frame))
    return chain, movable


class EeMonitor(Node):
    def __init__(self, args):
        super().__init__('ee_live_plot')
        self.args = args
        self.lock = threading.Lock()
        n = int(args.window * 250) + 100
        self.ref = collections.deque(maxlen=n)      # (t, x, y, z, v)  mm, mm/s
        self.meas = collections.deque(maxlen=n)     # (t, x, y, z, v)
        self.err = collections.deque(maxlen=n)      # (t, e)           mm
        self.sent = collections.deque(maxlen=n)     # (t, v)           mm/s
        self.t0 = None
        self.fk = None
        self.movable = []
        self.prev_ref = None
        self.meas_hist = collections.deque(maxlen=200)
        self.last_tr = None
        self.last_sent = float('nan')
        self.warned_names = False
        self.csv_writer = None
        self.csv_file = None
        if args.csv:
            self.csv_file = open(args.csv, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['t', 'x_ref', 'y_ref', 'z_ref', 'x_meas', 'y_meas', 'z_meas',
                                      'v_ref', 'v_meas', 'err', 'lin_spd_sent'])

        if args.urdf:
            with open(args.urdf) as f:
                self.set_urdf(f.read())
        else:
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, '/robot_description', lambda m: self.set_urdf(m.data), qos)

        self.create_subscription(JointTrajectoryControllerState, args.topic, self.on_state,
                                 QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Float32, '/comau/fly_lin_velocity_sent', self.on_sent, 10)

    def set_urdf(self, xml):
        if self.fk is not None:
            return
        robot = URDF.from_xml_string(xml)
        # Il solver tiene solo un riferimento C++ alla catena: va tenuta viva come attributo
        self.chain, movable = chain_from_urdf(robot, self.args.base, self.args.tip)
        self.fk = kdl.ChainFkSolverPos_recursive(self.chain)
        self.q = kdl.JntArray(self.chain.getNrOfJoints())
        self.movable = movable
        self.get_logger().info('Cinematica: %s -> %s, giunti %s'
                               % (self.args.base or robot.get_root(), self.args.tip, movable))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def flange(self, names, positions):
        index = {n: i for i, n in enumerate(names)}
        for k, name in enumerate(self.movable):
            if name not in index or index[name] >= len(positions):
                return None
            self.q[k] = positions[index[name]]
        frame = kdl.Frame()
        self.fk.JntToCart(self.q, frame)
        return np.array([frame.p.x(), frame.p.y(), frame.p.z()]) * 1000.0

    @staticmethod
    def speed(prev, t, p):
        if prev is None or t - prev[0] <= 1e-6:
            return 0.0
        return float(np.linalg.norm(p - prev[1]) / (t - prev[0]))

    def on_state(self, msg):
        if self.fk is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t == 0.0:
            t = self.now()
        p_ref = self.flange(msg.joint_names, msg.reference.positions)
        p_meas = self.flange(msg.joint_names, msg.feedback.positions)
        if p_ref is None or p_meas is None:
            if not self.warned_names:
                self.get_logger().warn('Giunti %s non trovati in %s' % (self.movable, list(msg.joint_names)))
                self.warned_names = True
            return
        with self.lock:
            if self.t0 is None:
                self.t0 = t
            tr = t - self.t0
            self.last_tr = tr
            v_ref = self.speed(self.prev_ref, tr, p_ref)
            self.prev_ref = (tr, p_ref)
            self.ref.append((tr, *p_ref, v_ref))
            # Velocità misurata su una finestra fissa (default 20 ms, il periodo dei pacchetti di stato):
            # tra due campioni a 10 ms la posizione spesso non cambia e la velocità alternerebbe 0 / doppia.
            # Non si scartano i campioni uguali, altrimenti gli stop veri (stop-and-go) sparirebbero.
            self.meas_hist.append((tr, p_meas))
            v_meas = float('nan')
            for t_old, p_old in reversed(self.meas_hist):
                if t_old <= tr - self.args.meas_dt + 0.002:
                    if tr - t_old > 1e-6:
                        v_meas = float(np.linalg.norm(p_meas - p_old) / (tr - t_old))
                    break
            self.meas.append((tr, *p_meas, v_meas))
            e = float(np.linalg.norm(p_ref - p_meas))
            self.err.append((tr, e))
            if self.csv_writer:
                self.csv_writer.writerow(['%.4f' % tr, *('%.3f' % c for c in p_ref), *('%.3f' % c for c in p_meas),
                                          '%.2f' % v_ref, '%.2f' % v_meas, '%.3f' % e, '%.2f' % self.last_sent])

    def on_sent(self, msg):
        # Stessa base dei tempi dello stato del controller (il messaggio non ha timestamp)
        with self.lock:
            if self.last_tr is None:
                return
            self.last_sent = msg.data * 1000.0
            self.sent.append((self.last_tr, self.last_sent))

    def snapshot(self):
        with self.lock:
            return (np.array(self.ref) if self.ref else np.empty((0, 5)),
                    np.array(self.meas) if self.meas else np.empty((0, 5)),
                    np.array(self.err) if self.err else np.empty((0, 2)),
                    np.array(self.sent) if self.sent else np.empty((0, 2)))

    def close(self):
        if self.csv_file:
            self.csv_file.close()


def build_plot(node, args):
    """Crea la figura e restituisce (fig, update) per FuncAnimation."""
    fig, axes = plt.subplots(5, 1, sharex=True, figsize=(11, 10),
                             gridspec_kw={'height_ratios': [1, 1, 1, 1.6, 1]})
    fig.canvas.manager.set_window_title('Comau end effector')
    labels = ['X [mm]', 'Y [mm]', 'Z [mm]']
    pos_lines = []
    for k in range(3):
        ref_line, = axes[k].plot([], [], '--', color='C%d' % k, lw=1, label='comandata')
        meas_line, = axes[k].plot([], [], '-', color='C%d' % k, lw=1.5, label='misurata')
        axes[k].set_ylabel(labels[k])
        axes[k].grid(True, alpha=0.3)
        pos_lines.append((ref_line, meas_line))
    axes[0].legend(loc='upper left', fontsize=8)
    v_ref_line, = axes[3].plot([], [], '--', color='C0', lw=1, label='comandata')
    v_meas_line, = axes[3].plot([], [], '-', color='C3', lw=1.2, label='misurata')
    v_sent_line, = axes[3].step([], [], where='post', color='C2', lw=1, alpha=0.8, label='lin_spd inviata')
    axes[3].set_ylabel('velocità [mm/s]')
    axes[3].legend(loc='upper left', fontsize=8)
    axes[3].grid(True, alpha=0.3)
    err_line, = axes[4].plot([], [], '-', color='C4', lw=1.2)
    axes[4].set_ylabel('errore [mm]')
    axes[4].set_xlabel('tempo [s]')
    axes[4].grid(True, alpha=0.3)
    title = fig.suptitle('In attesa di dati da %s ...' % args.topic)

    state = {'paused': False, 'status': ''}

    def set_lines(ref, meas, err, sent):
        for k in range(3):
            pos_lines[k][0].set_data(ref[:, 0], ref[:, 1 + k])
            pos_lines[k][1].set_data(meas[:, 0], meas[:, 1 + k])
        v_ref_line.set_data(ref[:, 0], ref[:, 4])
        v_meas_line.set_data(meas[:, 0], meas[:, 4])
        v_sent_line.set_data(sent[:, 0], sent[:, 1])
        err_line.set_data(err[:, 0], err[:, 1])

    def update(_):
        if state['paused']:
            # In pausa non si tocca niente: lo zoom e lo spostamento restano come li lascia l'utente
            return
        ref, meas, err, sent = node.snapshot()
        if len(ref) == 0:
            if node.fk is None:
                title.set_text('In attesa dell\'URDF su /robot_description (oppure usa --urdf)')
            return
        t_end = ref[-1, 0]
        t_start = max(t_end - args.window, ref[0, 0])
        # Solo la finestra visibile, così l'autoscala non tiene conto dei dati vecchi
        set_lines(ref[ref[:, 0] >= t_start], meas[meas[:, 0] >= t_start],
                  err[err[:, 0] >= t_start], sent[sent[:, 0] >= t_start])
        axes[0].set_xlim(t_start, max(t_end, t_start + 1e-3))
        for ax in axes:
            ax.relim()
            ax.autoscale_view(scalex=False)

        recent = meas[meas[:, 0] > t_end - 2.0, 4]
        recent = recent[np.isfinite(recent)]
        text = 'errore %.1f mm' % err[-1, 1] if len(err) else ''
        if len(sent):
            text += '   lin_spd %.0f mm/s' % sent[-1, 1]
        if len(recent) > 5 and recent.mean() > 5.0:
            text += '   v misurata (2 s): media %.0f mm/s, cv %.0f%%' % (recent.mean(),
                                                                         100.0 * recent.std() / recent.mean())
        state['status'] = text
        title.set_text(text)

    def toggle_pause(_event=None):
        state['paused'] = not state['paused']
        toolbar = getattr(fig.canvas, 'toolbar', None)
        if state['paused']:
            # Tutto lo storico in memoria (più della finestra), per poter scorrere anche indietro
            ref, meas, err, sent = node.snapshot()
            if len(ref):
                set_lines(ref, meas, err, sent)
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


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--urdf', help='file URDF (default: topic /robot_description)')
    parser.add_argument('--base', default='', help='link di base (default: radice dell\'URDF)')
    parser.add_argument('--tip', default='flange', help='link dell\'end effector (default: flange)')
    parser.add_argument('--topic', default='/joint_trajectory_controller/controller_state')
    parser.add_argument('--window', type=float, default=10.0, help='secondi mostrati (default 10)')
    parser.add_argument('--fps', type=float, default=15.0, help='aggiornamenti del grafico al secondo')
    parser.add_argument('--meas-dt', type=float, default=0.02,
                        help='finestra [s] per la velocità misurata (default 0.02, periodo dello stato)')
    parser.add_argument('--csv', help='salva anche i dati in questo file CSV')
    args = parser.parse_args()

    rclpy.init()
    node = EeMonitor(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    fig, update = build_plot(node, args)
    anim = FuncAnimation(fig, update, interval=1000.0 / args.fps, cache_frame_data=False)  # noqa: F841
    try:
        plt.show()
    finally:
        node.close()
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
