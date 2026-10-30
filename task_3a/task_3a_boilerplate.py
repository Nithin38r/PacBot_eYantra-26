"""PB Task 3A - controller (built on the official boilerplate).

Collect all three pellets, leave through either exit, never touch a wall,
never let the ghost touch us.

How it drives (micromouse style, sensors in the loop at all times):
  * STRAIGHT  : hold the heading with the gyro and stay centred on the side
                ToF sensors (fl / fr) directly.  Distance along the corridor
                comes from the encoders and is re-anchored by the front ToF
                (wall ahead) and by the exact moment a side wall starts or
                ends (that happens on a cell boundary).
  * TURN      : slow down to the cell centre, spin in place to the exact gyro
                angle, drive on.  No arcs that can clip a corner.
  * PLAN      : Dijkstra over (cell, heading) with real time costs for
                straight / turn / U-turn moves and a time-aware ghost cost.
                Every pellet order x both exits is tried.  The move out of a
                junction is decided BEFORE we reach it (while there is still
                room to brake), so the bot takes the safe branch early instead
                of driving in and reversing.
  * GHOST     : replanned at every cell; emergency stop + re-route if the
                ghost appears on the cells right ahead of us.  A ghost behind
                us is outrun, not avoided.

Run:   ./task_3a_launch            (terminal 1)
       python3 task_3a_boilerplate.py   (terminal 2)
Writes pacbot_log.csv (one line per sensor frame) for debugging.

Maze, protocol and geometry: see the official boilerplate docstring.
Cell (row, col) centre: x = col*0.22, y = (14-row)*0.22.  Headings E=0 N=1 W=2 S=3.
"""
import heapq
import itertools
import json
import math
import time
from collections import deque

import paho.mqtt.client as mqtt

from maze_3a import Maze

MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"
TOPIC_PELLETS = "pacbot/pellets"
TOPIC_GHOST = "pacbot/ghost"
TOPIC_RESULT = "pacbot/result"
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"

WHEEL_RADIUS = 0.017  # m
HALF_TRACK = 0.039    # m, wheel centre to centreline
WHEEL_LIMIT = 30.0    # rad/s, the simulator clamps to this
TICKS_PER_REV = 360   # encoder counts per wheel revolution
TICK_RAD = 2.0 * math.pi / TICKS_PER_REV   # radians of wheel turn per tick

# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------
N = 15
CELL = 0.22
HALF = CELL / 2.0
AXLE_OFFSET = 0.033          # axle is this far behind the chassis centre
M_PER_TICK = WHEEL_RADIUS * TICK_RAD
SPAWN_X, SPAWN_Y = -0.15, 0.0
EXIT_LINE = 3.19

# ---------------------------------------------------------------------------
# Tuning - everything that matters is here
# ---------------------------------------------------------------------------
V_MAX = 0.48          # m/s on clear straights (wheel limit is 0.51)
V_MIN = 0.05          # m/s, creeping speed at the very end of a stop
A_ACC = 2.0           # m/s^2 speeding up
A_DEC = 2.2           # m/s^2 slowing down before a turn / wall
WHEEL_SLEW = 160.0    # rad/s^2, max change of a wheel command (anti-slip)

HEAD_KP, HEAD_KI, HEAD_KD = 7.0, 0.0, 0.15   # heading PID while driving straight
K_LAT = 5.0           # rad per metre: lateral offset -> heading correction
LAT_MAX = 0.30        # rad, max heading correction for centring
W_PIVOT_MAX = 12.0    # rad/s, in-place turn rate (wheel limit allows 13)
SPIN_ACC = 40.0       # rad/s^2, spin braking (scaled down for slow motors)
SPIN_RATE_K = 1.5     # extra push when the measured turn rate lags the profile
BRAKE_K = 0.8         # active braking at the end of a stop (x measured speed)
PIVOT_LAG = 0.06      # s, motor lag allowed for when braking the spin
W_PIVOT_MIN = 0.8     # rad/s, never stall just short of the target
PIVOT_TOL = 0.04      # rad (~2 deg), the driving PID takes over from here

SIDE_WALL = 0.165     # m, side ToF below this = a wall in this cell
SIDE_MIN = 0.07       # m, closer than this to a side wall: push away hard
FRONT_STOP = 0.066    # m, front ToF below this: stop immediately
STOP_AT = 0.015       # m past the cell centre where the chassis stops to pivot
STOP_LAG = 0.06       # s, motor lag allowed for when braking to a stop
W_TEST = 14.0         # rad/s, wheel speed of the start-up motor test (0.24 m/s)
TEST_FRAMES = 60      # frames (0.6 s) of motor test
DECIDE_MARGIN = 0.05  # m, extra distance before a cell at which its move is decided

# Planner time costs (seconds)
C_STRAIGHT = 0.50
C_TURN = 0.85
C_UTURN = 1.10
PIVOT_COST = {0: 0.0, 1: 0.40, 2: 0.65, 3: 0.40}

# Ghost
GHOST_V_DEFAULT = 0.22    # m/s, assumed until measured
GHOST_HORIZON = 3.0       # s
GHOST_BLOCK_GAP = 0.16    # m
GHOST_SOFT_GAP = 0.50     # m
GHOST_STOP_DIST = 0.32    # m, ghost this close to a cell just ahead -> stop & re-route

LOG_FILE = "pacbot_log.csv"   # None to disable

DIR_UNIT = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]   # (dx, dy) for E N W S
DIR_RC = [(0, 1), (-1, 0), (0, -1), (1, 0)]                      # (drow, dcol)

# Official maze exactly as drawn in the task documentation (S spawn, E exits).
MAZE_ASCII = [
    "+---+---+---+---+---+---+---+ E +---+---+---+---+---+---+---+",
    "|                           |       |       |               |",
    "+   +---+---+   +---+   +   +   +   +   +   +   +   +   +   +",
    "|   |               |   |       |       |   |       |       |",
    "+   +   +   +---+   +   +   +   +---+   +   +   +---+---+   +",
    "|   |   |           |       |       |   |               |   |",
    "+   +   +   +---+   +   +   +---+   +   +---+   +   +   +   +",
    "|       |               |           |           |   |       |",
    "+   +---+   +---+---+   +   +---+   +---+   +---+   +   +---+",
    "|   |               |   |               |                   |",
    "+   +   +---+   +   +   +---+---+---+   +---+   +   +---+   +",
    "|       |       |   |               |           |           |",
    "+   +---+   +---+   +   +---+---+   +   +---+---+   +---+   +",
    "|       |   |       |           |       |           |       |",
    "+---+   +   +   +---+---+---+   +---+---+   +   +---+   +---+",
    "|                           | G             |               E",
    "+   +---+---+   +---+---+   +   +   +---+---+   +   +---+   +",
    "|           |                   |               |           |",
    "+   +   +   +   +---+---+---+   +---+   +   +---+   +---+---+",
    "|   |   |       |                   |   |                   |",
    "+   +   +---+   +   +---+   +---+   +   +---+---+   +---+   +",
    "|           |           |           |           |   |       |",
    "+   +---+   +---+---+   +   +---+---+   +   +   +   +   +   +",
    "|       |           |                   |   |   |   |   |   |",
    "+---+   +   +---+   +---+---+   +---+   +   +   +   +   +   +",
    "|       |               |           |       |           |   |",
    "+   +   +---+   +---+   +   +---+   +---+   +   +---+   +   +",
    "|   |           |       |       |               |           |",
    "+   +   +   +---+   +   +---+   +   +   +---+   +   +---+   +",
    "S       |           |                                       |",
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+",
]


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def cell_xy(cell):
    r, c = cell
    return c * CELL, (N - 1 - r) * CELL


def xy_cell(x, y):
    return (N - 1 - int(round(y / CELL)), int(round(x / CELL)))


def in_maze(cell):
    return 0 <= cell[0] < N and 0 <= cell[1] < N


# ===========================================================================
# Maze
# ===========================================================================
class MazeMap:
    """Wall map. open[r][c][d] is True when there is no wall on side d."""

    EXITS = {((7, 14), 0), ((0, 7), 1)}      # (cell, direction) pairs that leave the maze

    def __init__(self, ascii_rows=MAZE_ASCII, official=None):
        rows = [ln.ljust(4 * N + 1) for ln in ascii_rows]
        self.open = [[[False] * 4 for _ in range(N)] for _ in range(N)]
        for r in range(N):
            top, mid, bot = rows[2 * r], rows[2 * r + 1], rows[2 * r + 2]
            for c in range(N):
                o = self.open[r][c]
                o[1] = top[4 * c + 1:4 * c + 4] != "---"
                o[3] = bot[4 * c + 1:4 * c + 4] != "---"
                o[2] = mid[4 * c] != "|"
                o[0] = mid[4 * c + 4] != "|"
        if official is not None:
            self._check_official(official)
        # sanity: both sides of every inner wall agree
        for r in range(N):
            for c in range(N - 1):
                assert self.open[r][c][0] == self.open[r][c + 1][2], (r, c)
        for r in range(N - 1):
            for c in range(N):
                assert self.open[r][c][3] == self.open[r + 1][c][1], (r, c)
        # flat neighbour table for the planner: nbr[idx][d] = idx2, -1 wall, -2 exit
        self.nbr = []
        for r in range(N):
            for c in range(N):
                row = []
                for d in range(4):
                    if not self.open[r][c][d]:
                        row.append(-1)
                        continue
                    nr, nc = r + DIR_RC[d][0], c + DIR_RC[d][1]
                    if in_maze((nr, nc)):
                        row.append(nr * N + nc)
                    else:
                        row.append(-2 if ((r, c), d) in self.EXITS else -1)
                self.nbr.append(row)

    def _check_official(self, om):
        """Use maze_3a.Maze.official() as the authority for inner walls."""
        dcode = [getattr(om, "EAST", 0), getattr(om, "NORTH", 1),
                 getattr(om, "WEST", 2), getattr(om, "SOUTH", 3)]
        diff = 0
        for r in range(N):
            for c in range(N):
                for d in range(4):
                    nr, nc = r + DIR_RC[d][0], c + DIR_RC[d][1]
                    if not in_maze((nr, nc)):
                        continue          # border gaps handled by EXITS / spawn
                    try:
                        o = bool(om.can_move((r, c), dcode[d]))
                    except Exception:
                        return
                    if o != self.open[r][c][d]:
                        diff += 1
                        self.open[r][c][d] = o
        print("[maze] official maze matches the built-in copy" if diff == 0
              else f"[maze] WARNING: {diff} wall sides taken from Maze.official()")

    def is_open(self, cell, d):
        if not in_maze(cell):
            return False
        return self.open[cell[0]][cell[1]][d]

    def wall_distance(self, cell, d):
        """Distance from the cell centre to the first wall face line in direction d
        (centre-line of the wall, i.e. half thickness not included)."""
        dist = HALF
        cur = cell
        while self.is_open(cur, d):
            nxt = (cur[0] + DIR_RC[d][0], cur[1] + DIR_RC[d][1])
            if not in_maze(nxt):
                return 9.0
            cur = nxt
            dist += CELL
        return dist

    def bfs(self, sources):
        """Graph distance in cells from a set of cells."""
        dist = [99] * (N * N)
        q = deque()
        for cell in sources:
            if in_maze(cell):
                i = cell[0] * N + cell[1]
                if dist[i] != 0:
                    dist[i] = 0
                    q.append(i)
        while q:
            i = q.popleft()
            for j in self.nbr[i]:
                if j >= 0 and dist[j] > dist[i] + 1:
                    dist[j] = dist[i] + 1
                    q.append(j)
        return dist


# ===========================================================================
# PID
# ===========================================================================
class PID:
    def __init__(self, kp, ki, kd, i_limit=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit = i_limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev = None

    def update(self, error, dt):
        if dt <= 0:
            dt = 0.01
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral + error * dt))
        deriv = 0.0 if self.prev is None else (error - self.prev) / dt
        self.prev = error
        return self.kp * error + self.ki * self.integral + self.kd * deriv


# ===========================================================================
# Planner
# ===========================================================================
class Planner:
    """Dijkstra on (cell, heading) with time-based costs and a ghost cost."""

    def __init__(self, maze):
        self.maze = maze
        self.move_cost = {0: C_STRAIGHT, 1: C_TURN, 3: C_TURN, 2: C_UTURN}

    # ---- ghost model -------------------------------------------------------
    def set_ghost(self, ghost_cells, ghost_v, strict=True):
        self.gdist = self.maze.bfs(ghost_cells) if ghost_cells else [99] * (N * N)
        self.gv = ghost_v
        self.strict = strict

    def ghost_cost(self, idx, tau):
        dg = self.gdist[idx]
        if dg >= 99:
            return 0.0
        if self.strict:
            if tau < GHOST_HORIZON:
                # how much corridor would still separate us if the ghost came
                # straight for this cell while we drive to it
                gap = dg * CELL - self.gv * tau
                if dg <= 1 or (gap < GHOST_BLOCK_GAP and tau < 1.5):
                    return math.inf
                if gap < GHOST_SOFT_GAP:
                    return 2.5 * (GHOST_SOFT_GAP - gap) / GHOST_SOFT_GAP
                return 0.0
            return 1.0 if dg <= 1 else (0.3 if dg == 2 else 0.0)
        # relaxed: only the ghost's own cell and its neighbours in the near future
        if tau < 1.5 and dg <= 1:
            return math.inf
        return 3.0 if dg <= 1 else (1.0 if dg == 2 else 0.0)

    # ---- single-source Dijkstra --------------------------------------------
    def dijkstra(self, starts, tau0):
        """starts: list of (cell, heading, cost). Returns (dist, parent) over idx*4+k."""
        S = N * N * 4
        dist = [math.inf] * S
        parent = [-1] * S
        pq = []
        for cell, k, c0 in starts:
            s = (cell[0] * N + cell[1]) * 4 + k
            if c0 < dist[s]:
                dist[s] = c0
                parent[s] = -1
                heapq.heappush(pq, (c0, s))
        nbr = self.maze.nbr
        mc = self.move_cost
        while pq:
            d, s = heapq.heappop(pq)
            if d > dist[s]:
                continue
            idx, k = divmod(s, 4)
            for nd in range(4):
                j = nbr[idx][nd]
                if j < 0:
                    continue
                nc = d + mc[(nd - k) % 4]
                g = self.ghost_cost(j, tau0 + nc)
                if g == math.inf:
                    continue
                nc += g
                t = j * 4 + nd
                if nc < dist[t]:
                    dist[t] = nc
                    parent[t] = s
                    heapq.heappush(pq, (nc, t))
        return dist, parent

    @staticmethod
    def rebuild(parent, s):
        out = []
        while s != -1:
            out.append(s)
            s = parent[s]
        out.reverse()
        return [((i // 4) // N, (i // 4) % N) for i in out], out[-1] % 4

    # ---- full route --------------------------------------------------------
    def plan(self, starts, pellets, tau0=0.0):
        """Best route from starts through every pellet to an exit.
        Returns (cells, cost) with cells[0] = start cell, last cell = exit cell,
        or (None, inf)."""
        pellets = [tuple(p) for p in pellets]
        cache = {}

        def run(st, tau):
            key = (tuple(st), round(tau, 1))
            if key not in cache:
                cache[key] = self.dijkstra(st, tau)
            return cache[key]

        best_cost, best_cells = math.inf, None
        orders = list(itertools.permutations(pellets)) if pellets else [()]
        for order in orders:
            st = starts
            tau = tau0
            total = 0.0
            cells = None
            got = set()
            ok = True
            for p in order:
                if p in got:
                    continue
                dist, parent = run(st, tau)
                base = (p[0] * N + p[1]) * 4
                k = min(range(4), key=lambda kk: dist[base + kk])
                if dist[base + k] == math.inf:
                    ok = False
                    break
                seg, _ = self.rebuild(parent, base + k)
                total += dist[base + k] - (0 if cells is None else 0)
                cells = seg if cells is None else cells + seg[1:]
                got.update(c for c in seg if c in pellets)
                tau = tau0 + total
                st = [(p, k, 0.0)]
                if total >= best_cost:
                    ok = False
                    break
            if not ok:
                continue
            # leave through the cheaper exit
            dist, parent = run(st, tau)
            best_e = None
            for (ecell, ed) in MazeMap.EXITS:
                base = (ecell[0] * N + ecell[1]) * 4
                for k in range(4):
                    c = dist[base + k] + self.move_cost[(ed - k) % 4]
                    if best_e is None or c < best_e[0]:
                        best_e = (c, base + k)
            if best_e is None or best_e[0] == math.inf:
                continue
            seg, _ = self.rebuild(parent, best_e[1])
            total += best_e[0]
            cells = seg if cells is None else cells + seg[1:]
            if total < best_cost:
                best_cost, best_cells = total, cells
        return best_cells, best_cost



# ===========================================================================
# Driving
# ===========================================================================
def move_dir(a, b):
    return DIR_RC.index((b[0] - a[0], b[1] - a[1]))


def exit_dir(cell):
    for c, d in MazeMap.EXITS:
        if c == cell:
            return d
    return None


class GridBot:
    """Cell-by-cell driver with the ToF sensors in the loop."""

    def __init__(self, official_maze=None):
        self.maze = MazeMap(official=official_maze)
        self.planner = Planner(self.maze)
        # pose of the chassis centre (world frame)
        self.x, self.y, self.th = SPAWN_X, SPAWN_Y, 0.0
        self.prev_ticks = None
        self.v_meas = 0.0
        self.w_meas = 0.0
        self.gyro_bias = 0.0
        self.calib = []
        self.side_ref = HALF            # side reading when centred (learned)
        self._side_acc = []
        self.prev_side = [None, None]   # wall seen left / right last frame
        self.last = None
        # world
        self.pellets = None
        self.pellet_version = 0
        self._ghost = None
        self.ghost_hist = deque(maxlen=60)
        self.ghost_v = GHOST_V_DEFAULT
        self.sim_t = 0.0
        # motion
        self.mode = "WAIT"
        self.here = (14, -1)            # cell whose centre our distance is measured from
        self.k = 0                      # heading 0..3
        self.route = []                 # [here, next, ...], last cell is an exit cell
        self.decided_for = None         # cell whose outgoing move is already decided
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.wl = 0.0
        self.wr = 0.0
        self.pivot_target = 0.0
        self.pivot_ok = 0
        self.after_pivot = None
        self.hold_until = 0.0
        self.finished = False
        self.head_pid = PID(HEAD_KP, HEAD_KI, HEAD_KD)
        self.tau = 0.03
        self.gain = 1.0
        self.stop_lag = STOP_LAG
        self.pivot_lag = PIVOT_LAG
        self.w_pivot_max = W_PIVOT_MAX
        self.last_gkey = None
        self.last_pv = -1

    # ------------------------------------------------------------- ghost --
    @property
    def ghost(self):
        return self._ghost

    @ghost.setter
    def ghost(self, g):
        self._ghost = g
        if g is not None:
            self.ghost_hist.append((time.monotonic(), g["x"], g["y"]))

    def ghost_cells(self):
        g = self._ghost
        if g is None:
            return []
        cells = {(g["row"], g["col"])}
        cx, cy = cell_xy((g["row"], g["col"]))
        dx, dy = g["x"] - cx, g["y"] - cy
        if abs(dx) > 0.02:
            cells.add((g["row"], g["col"] + (1 if dx > 0 else -1)))
        if abs(dy) > 0.02:
            cells.add((g["row"] + (-1 if dy > 0 else 1), g["col"]))
        return sorted(c for c in cells if in_maze(c))

    def update_ghost_speed(self):
        h = self.ghost_hist
        if len(h) < 30:
            return
        t0, x0, y0 = h[0]
        t1, x1, y1 = h[-1]
        if t1 - t0 > 0.1:
            v = math.hypot(x1 - x0, y1 - y0) / (t1 - t0)
            if v > self.ghost_v:
                self.ghost_v = min(0.6, 0.5 * self.ghost_v + 0.5 * v)

    # ---------------------------------------------------------- geometry --
    def rel(self):
        """(along, lateral) of the chassis relative to centre(here), heading k."""
        cx, cy = cell_xy(self.here)
        hx, hy = DIR_UNIT[self.k]
        dx, dy = self.x - cx, self.y - cy
        return dx * hx + dy * hy, -dx * hy + dy * hx

    def stop_distance(self):
        """Along-distance (from centre(here)) where we must stop, inf = drive on."""
        r = self.route
        m = 0
        while m + 1 < len(r) and move_dir(r[m], r[m + 1]) == self.k:
            m += 1
        if m + 1 < len(r):
            return m * CELL + STOP_AT               # turn / U-turn at r[m]
        ed = exit_dir(r[m]) if r else None
        if ed == self.k:
            return math.inf                         # straight out of the exit
        return m * CELL + STOP_AT                   # turn towards the exit at r[m]

    # ---------------------------------------------------------- planning --
    def plan(self, starts, tau0=0.0):
        gc = self.ghost_cells()
        for strict in (True, False):
            self.planner.set_ghost(gc, self.ghost_v, strict)
            cells, _ = self.planner.plan(starts, self.pellets or [], tau0)
            if cells:
                return cells
        return None

    def flee(self, cell):
        """No safe route: drive to the reachable cell where we beat the ghost by
        the widest margin (never stand still while a ghost closes in)."""
        gd = self.maze.bfs(self.ghost_cells())
        if gd[cell[0] * N + cell[1]] >= 99:
            return None
        start = cell[0] * N + cell[1]
        prev = {start: -1}
        dist = {start: 0}
        q = deque([start])
        while q:
            i = q.popleft()
            if dist[i] >= 7:
                continue
            for j in self.maze.nbr[i]:
                if j >= 0 and j not in prev and gd[j] >= 2:
                    prev[j] = i
                    dist[j] = dist[i] + 1
                    q.append(j)
        best, best_m = None, -math.inf
        vg = max(self.ghost_v, 0.1)
        for j, dme in dist.items():
            if dme == 0:
                continue
            margin = gd[j] * CELL / vg - dme * C_STRAIGHT
            if margin > best_m:
                best, best_m = j, margin
        if best is None:
            return None
        path = []
        j = best
        while j != -1:
            path.append((j // N, j % N))
            j = prev[j]
        path.reverse()
        return path

    def decide_ahead(self):
        """Decide the move out of the next cell while there is room to brake."""
        if len(self.route) < 2:
            return
        nxt = self.route[1]
        along, _ = self.rel()
        tau0 = max(0.0, CELL - along) / max(self.v_cmd, 0.2)
        cells = self.plan([(nxt, self.k, 0.0)], tau0)
        if cells:
            self.route = [self.here] + cells
        self.decided_for = nxt
        self.last_gkey = tuple(self.ghost_cells())
        self.last_pv = self.pellet_version

    def plan_at_rest(self):
        """Stopped: plan freely (any heading), then pivot or drive."""
        along, _ = self.rel()
        if along > HALF and len(self.route) > 1:      # we are already in the next cell
            self.here = self.route[1]
        cell = self.here
        if not in_maze(cell):
            cells = self.plan([((14, 0), 0, 0.0)])
            self.route = [(14, -1)] + (cells or [(14, 0)])
            self.start_straight()
            return
        starts = [(cell, kk, PIVOT_COST[(kk - self.k) % 4]) for kk in range(4)]
        cells = self.plan(starts)
        if not cells:
            cells = self.flee(cell)
            if cells:
                print(f"[{self.sim_t:6.2f}s] no safe route -> escaping to {cells[-1]}")
        if cells and len(cells) == 1 and exit_dir(cells[0]) is not None:
            # standing in the exit cell: face the exit and drive out
            d = exit_dir(cells[0])
            self.route = cells
            self.decided_for = None
            if d == self.k:
                self.start_straight()
            else:
                self.start_pivot(d)
            return
        if not cells or len(cells) < 2:
            # boxed in: wait a moment, then look again
            self.mode = "HOLD"
            self.hold_until = self.sim_t + 0.1
            return
        d = move_dir(cells[0], cells[1]) if in_maze(cells[1]) else exit_dir(cells[0])
        self.route = cells
        self.decided_for = cells[1] if len(cells) > 1 else None
        self.last_gkey = tuple(self.ghost_cells())
        self.last_pv = self.pellet_version
        if d == self.k:
            self.start_straight()
        else:
            self.start_pivot(d)

    def start_straight(self):
        self.mode = "STRAIGHT"
        self.head_pid.reset()

    def start_pivot(self, d):
        delta = (d - self.k) % 4
        turn = {1: math.pi / 2, 3: -math.pi / 2, 2: math.pi}[delta]
        if delta == 2:
            # spin towards the side with more room (larger side reading)
            f = self.last
            if f is not None and f["fr"] > f["fl"]:
                turn = -math.pi
        self.pivot_target = wrap(self.k * math.pi / 2 + turn)
        self.after_pivot = d
        self.pivot_ok = 0
        self.mode = "PIVOT"

    # ------------------------------------------------------ localisation --
    def odometry(self, f):
        tl, tr = f["ticks_l"], f["ticks_r"]
        dt = f.get("dt", 0.01) or 0.01
        if self.prev_ticks is None:
            self.prev_ticks = (tl, tr)
            return
        dl = (tl - self.prev_ticks[0]) * M_PER_TICK
        dr = (tr - self.prev_ticks[1]) * M_PER_TICK
        self.prev_ticks = (tl, tr)
        ds = 0.5 * (dl + dr)
        dth = (f["gyro"][2] - self.gyro_bias) * dt
        ax = self.x - AXLE_OFFSET * math.cos(self.th)
        ay = self.y - AXLE_OFFSET * math.sin(self.th)
        thm = self.th + 0.5 * dth
        ax += ds * math.cos(thm)
        ay += ds * math.sin(thm)
        self.th = wrap(self.th + dth)
        self.x = ax + AXLE_OFFSET * math.cos(self.th)
        self.y = ay + AXLE_OFFSET * math.sin(self.th)
        self.v_meas = 0.7 * self.v_meas + 0.3 * (ds / dt)
        self.w_meas = 0.7 * self.w_meas + 0.3 * (dth / dt)

    def correct(self, f):
        """Re-anchor the pose on the walls the map says are there."""
        k = int(round(self.th / (math.pi / 2))) % 4
        herr = wrap(self.th - k * math.pi / 2)
        if abs(herr) > 0.10:
            self.prev_side = [None, None]
            return
        cell = xy_cell(self.x, self.y)
        if not in_maze(cell):
            self.prev_side = [None, None]
            return
        cx, cy = cell_xy(cell)
        hx, hy = DIR_UNIT[k]
        lx, ly = -hy, hx
        a = (self.x - cx) * hx + (self.y - cy) * hy       # along, in this cell
        lat = (self.x - cx) * lx + (self.y - cy) * ly
        c = math.cos(herr)
        fl, fr = f["fl"] * c, f["fr"] * c
        left_d, right_d = (k + 1) % 4, (k + 3) % 4
        wl = not self.maze.is_open(cell, left_d)
        wr = not self.maze.is_open(cell, right_d)

        # 1. lateral, straight from the side sensors
        if abs(a) < 0.085:
            meas = None
            if wl and wr and fl < SIDE_WALL and fr < SIDE_WALL:
                meas = 0.5 * (fr - fl)
                if abs(meas) < 0.015:
                    self._side_acc.append(0.5 * (fl + fr))
                    if len(self._side_acc) > 300:
                        self._side_acc = self._side_acc[-300:]
                    if len(self._side_acc) >= 20:
                        s = sorted(self._side_acc)
                        self.side_ref = s[len(s) // 2]
            elif wl and fl < SIDE_WALL:
                meas = self.side_ref - fl
            elif wr and fr < SIDE_WALL:
                meas = fr - self.side_ref
            if meas is not None and abs(meas - lat) < 0.08:
                e = meas - lat
                self.x += 0.3 * e * lx
                self.y += 0.3 * e * ly

        # 2. along, from the wall ahead
        sl, sr = f["sl"] * c, f["sr"] * c
        if abs(sl - sr) < 0.02:
            fm = 0.5 * (sl + sr)
            wall = self.maze.wall_distance(cell, k)
            if fm < 0.6 and wall < 1.5:
                e = fm - (wall - a)              # > 0: we are further back than we think
                if abs(e) < 0.08:
                    self.x -= 0.3 * e * hx
                    self.y -= 0.3 * e * hy

        # 3. along, from a side wall starting / ending (= a cell boundary)
        seen = [f["fl"] < SIDE_WALL, f["fr"] < SIDE_WALL]
        if abs(a) > 0.06 and abs(self.v_meas) > 0.03:
            b = HALF if a > 0 else -HALF
            sg = 1 if a > 0 else -1
            nb = (cell[0] + DIR_RC[k][0] * sg, cell[1] + DIR_RC[k][1] * sg)
            for i, side in enumerate((left_d, right_d)):
                prev = self.prev_side[i]
                if prev is None or prev == seen[i] or not in_maze(nb):
                    continue
                if (not self.maze.is_open(cell, side)) != (not self.maze.is_open(nb, side)) \
                        and abs(a - b) < 0.04:
                    self.x += 0.6 * (b - a) * hx
                    self.y += 0.6 * (b - a) * hy
        self.prev_side = seen

    def calibrate(self):
        n = len(self.calib)
        self.gyro_bias = sum(c["gyro"][2] for c in self.calib) / n
        print(f"[calib] gyro bias {self.gyro_bias:+.5f} rad/s, "
              f"front ToF {sorted(0.5 * (c['sl'] + c['sr']) for c in self.calib)[n // 2]:.3f} m "
              f"(expected 0.480)")

    def finish_motor_test(self, dt):
        """Fit the wheel response: angle(T) = W*(T - tau*(1-exp(-T/tau)))."""
        th = self.test_frames
        T = len(th) * dt
        deficit = W_TEST * T - th[-1]
        tau = max(0.02, min(0.6, deficit / W_TEST))
        k = 15
        w_end = (th[-1] - th[-1 - k]) / (k * dt)
        # speed still rising at the end of the window -> correct for it
        gain = max(0.5, min(1.3, w_end / (W_TEST * (1.0 - math.exp(-T / tau)))))
        self.tau = tau
        self.gain = gain
        self.stop_lag = tau + 0.03
        self.pivot_lag = tau + 0.03
        kp = min(HEAD_KP, 0.9 / tau)
        self.head_pid = PID(kp, HEAD_KI, HEAD_KD * min(1.0, 0.08 / tau) + 0.05)
        self.w_pivot_max = W_PIVOT_MAX
        self.v_cmd = w_end * WHEEL_RADIUS
        print(f"[motors] response lag {tau * 1000:.0f} ms, speed gain {gain:.2f} -> "
              f"heading Kp {kp:.1f}, stop/spin lead {self.stop_lag * 1000:.0f} ms, "
              f"spin max {self.w_pivot_max:.1f} rad/s")
        self.start_straight()

    # ------------------------------------------------------------ control --
    def control(self, f):
        """One sensor frame in, (left, right) rad/s out."""
        self.last = f
        self.sim_t = f.get("t", self.sim_t)
        dt = f.get("dt", 0.01) or 0.01
        if self.mode == "WAIT":
            self.calib.append(f)
            if len(self.calib) >= 120 and self.pellets is not None and self._ghost is not None:
                self.calibrate()
                self.prev_ticks = (f["ticks_l"], f["ticks_r"])
                t0 = time.monotonic()
                cells = self.plan([((14, 0), 0, 0.0)])
                self.route = [(14, -1)] + (cells or [(14, 0)])
                self.decided_for = (14, 0)
                print(f"[plan] pellets {self.pellets} -> {len(self.route) - 1} cells "
                      f"({(time.monotonic() - t0) * 1000:.0f} ms): {self.route[1:]}")
                # motor self-test: a fixed wheel-speed step down the entry chute
                self.mode = "MOTORTEST"
                self.test_frames = []
                self.test_t0 = (f["ticks_l"], f["ticks_r"])
                self.wl = self.wr = W_TEST
                return W_TEST, W_TEST
            return 0.0, 0.0
        self.odometry(f)
        if self.mode == "MOTORTEST":
            self.test_frames.append(0.5 * ((f["ticks_l"] - self.test_t0[0]) +
                                           (f["ticks_r"] - self.test_t0[1])) * TICK_RAD)
            n = len(self.test_frames)
            if n < TEST_FRAMES:
                self.v_cmd = W_TEST * WHEEL_RADIUS
                w = self.heading_hold(dt)
                corr = w * HALF_TRACK / WHEEL_RADIUS
                self.wl, self.wr = W_TEST - corr, W_TEST + corr
                return self.wl, self.wr
            self.finish_motor_test(dt)
        self.correct(f)
        self.update_ghost_speed()
        if self.finished or self.x > EXIT_LINE + 0.10 or self.y > EXIT_LINE + 0.10:
            self.finished = True
            return 0.0, 0.0

        v, w = 0.0, 0.0
        if self.mode == "STRAIGHT":
            v, w = self.straight(f, dt)
        elif self.mode == "BRAKE":
            self.v_cmd = max(0.0, self.v_cmd - 1.5 * A_DEC * dt)
            v, w = self.v_cmd, self.heading_hold(dt)
            if self.v_cmd == 0.0 and abs(self.v_meas) < 0.02:
                self.plan_at_rest()
        elif self.mode == "HOLD":
            if self.sim_t >= self.hold_until:
                self.plan_at_rest()
        if self.mode == "PIVOT":
            v, w = 0.0, self.pivot(dt)
        self.w_cmd = w
        return self.wheels(v, w, dt)

    def heading_hold(self, dt):
        _, lat = self.rel()
        target = self.k * math.pi / 2 - max(-LAT_MAX, min(LAT_MAX, K_LAT * lat))
        return self.head_pid.update(wrap(target - self.th), dt)

    def straight(self, f, dt):
        along, lat = self.rel()
        # passed the centre of the next cell -> it becomes `here`
        while len(self.route) > 1 and along >= CELL - 1e-3:
            self.here = self.route[1]
            self.route = self.route[1:]
            along, lat = self.rel()
        # decide the move out of the next cell before we are too close to it
        brake = self.v_cmd ** 2 / (2 * A_DEC) + DECIDE_MARGIN
        if len(self.route) > 1 and self.decided_for != self.route[1] and \
                along >= CELL + STOP_AT - brake - 0.06:
            self.decide_ahead()
        elif (tuple(self.ghost_cells()) != self.last_gkey or self.pellet_version != self.last_pv) \
                and len(self.route) > 2 and self.decided_for == self.route[1]:
            # world changed: re-decide from the cell after the next one
            nxt2 = self.route[2]
            if in_maze(nxt2):
                tau0 = max(0.0, 2 * CELL - along) / max(self.v_cmd, 0.2)
                cells = self.plan([(nxt2, move_dir(self.route[1], nxt2), 0.0)], tau0)
                if cells:
                    self.route = self.route[:2] + cells
            self.last_gkey = tuple(self.ghost_cells())
            self.last_pv = self.pellet_version
        if self.ghost_ahead():
            print(f"[{self.sim_t:6.2f}s] ghost ahead -> stop and re-route")
            self.mode = "BRAKE"
            return self.v_cmd, self.heading_hold(dt)

        stop = self.stop_distance()
        remaining = stop - along
        if stop != math.inf:
            # stopping in front of a wall: trust the front sensor over the encoders
            front = 0.5 * (f["sl"] + f["sr"])
            if abs(f["sl"] - f["sr"]) < 0.03 and front < 0.5:
                stop_cell_wall = (stop - STOP_AT) + HALF          # wall plane, from centre(here)
                if abs((stop_cell_wall - along) - front) < 0.06:  # it is the wall we expect
                    remaining = min(remaining, front - (HALF - STOP_AT))
            # brake early by the motor lag
            remaining -= max(0.0, self.v_meas) * self.stop_lag
        if remaining <= 0.003:
            self.v_cmd = 0.0
            if abs(self.v_meas) < 0.04:
                self.plan_at_rest()
                return 0.0, 0.0
            # still rolling: brake actively instead of coasting on slow motors
            return -BRAKE_K * self.v_meas, self.heading_hold(dt)
        v_ref = min(V_MAX, max(V_MIN, math.sqrt(2.0 * A_DEC * remaining)))
        if v_ref > self.v_cmd:
            self.v_cmd = min(v_ref, self.v_cmd + A_ACC * dt)
        else:
            self.v_cmd = v_ref
        v = self.v_cmd
        # reactive guards
        front = min(f["sl"], f["sr"])
        if front < FRONT_STOP:
            v = 0.0
            self.v_cmd = 0.0
        w = self.heading_hold(dt)
        if f["fl"] < SIDE_MIN:
            w -= 6.0 * (SIDE_MIN - f["fl"])
        if f["fr"] < SIDE_MIN:
            w += 6.0 * (SIDE_MIN - f["fr"])
        return v, w

    def ghost_ahead(self):
        g = self._ghost
        if g is None or self.sim_t < self.hold_until:
            return False
        gx, gy = g["x"], g["y"]
        hx, hy = DIR_UNIT[self.k]
        if (gx - self.x) * hx + (gy - self.y) * hy < 0.0:
            return False                       # behind us: just keep going
        gd = self.maze.bfs(self.ghost_cells())
        stop = self.stop_distance()
        along, _ = self.rel()
        for j in range(1, min(3, len(self.route))):
            c = self.route[j]
            if not in_maze(c) or j * CELL > min(stop, along + 0.5):
                break
            if gd[c[0] * N + c[1]] <= 1:
                cx, cy = cell_xy(c)
                if math.hypot(gx - cx, gy - cy) < GHOST_STOP_DIST:
                    self.hold_until = self.sim_t + 0.5
                    return True
        return False

    def pivot(self, dt):
        """Fast in-place turn: a turn-rate profile (accelerate, cruise, brake to
        zero exactly on the target angle) plus a rate loop that pushes harder
        when slow motors lag behind the profile (it also brakes actively)."""
        e = wrap(self.pivot_target - self.th)
        if abs(e) < PIVOT_TOL and abs(self.w_meas) < 1.5:
            self.k = self.after_pivot
            self.start_straight()
            return 0.0
        acc = SPIN_ACC / (1.0 + 4.0 * self.tau)        # gentler braking for slow motors
        w_des = math.copysign(min(self.w_pivot_max, math.sqrt(2.0 * acc * abs(e))), e)
        if abs(w_des) < W_PIVOT_MIN:
            w_des = math.copysign(W_PIVOT_MIN, e)
        return w_des + SPIN_RATE_K * (w_des - self.w_meas)

    def wheels(self, v, w, dt):
        wl = (v - w * HALF_TRACK) / WHEEL_RADIUS / self.gain
        wr = (v + w * HALF_TRACK) / WHEEL_RADIUS / self.gain
        m = max(abs(wl), abs(wr))
        if m > WHEEL_LIMIT * 0.97:
            wl *= WHEEL_LIMIT * 0.97 / m
            wr *= WHEEL_LIMIT * 0.97 / m
        dmax = (WHEEL_SLEW if self.mode != "PIVOT" else 3.0 * WHEEL_SLEW) * dt
        wl = self.wl + max(-dmax, min(dmax, wl - self.wl))
        wr = self.wr + max(-dmax, min(dmax, wr - self.wr))
        self.wl, self.wr = wl, wr
        return wl, wr


# ===========================================================================
# Official boilerplate structure (names kept exactly as shipped)
# ===========================================================================
def _mqtt_client():
    # paho-mqtt >= 2.0 requires picking a callback API version explicitly.
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def wheels(forward, turn):
    """Differential drive mixer: +turn steers LEFT (counter-clockwise)."""
    return forward - turn, forward + turn


class Controller(GridBot):
    def __init__(self, maze):
        super().__init__(official_maze=maze)
        self.have_pellets = False
        self.elapsed = 0.0
        self._log = None
        self._n = 0
        if LOG_FILE:
            try:
                self._log = open(LOG_FILE, "w")
                self._log.write("t,mode,x,y,th,here,k,v_cmd,w_cmd,left,right,fl,fr,sl,sr,"
                                "ticks_l,ticks_r,gyro_z,ghost_x,ghost_y\n")
            except OSError:
                self._log = None

    def on_pellets(self, msg):
        """Called whenever the remaining-pellet list arrives or changes.

        msg is {"pellets": [[row, col], ...]}.
        """
        self.pellets = [tuple(c) for c in msg.get("pellets", [])]
        self.pellet_version += 1
        self.have_pellets = True
        print(f"[{self.sim_t:6.2f}s] pellets remaining: {self.pellets}")

    def decide(self, s):
        """One sensor frame in, one wheel command out.

        `s` is the decoded pacbot/sensors payload. Return (left, right) in
        rad/s.
        """
        self.elapsed += s["dt"]
        if not self.have_pellets:
            return 0.0, 0.0
        left, right = self.control(s)
        if self._log is not None:
            g = self._ghost or {"x": 0.0, "y": 0.0}
            self._log.write(
                f"{s.get('t', 0):.3f},{self.mode},{self.x:.4f},{self.y:.4f},{self.th:.4f},"
                f"{self.here[0]}:{self.here[1]},{self.k},{self.v_cmd:.3f},{self.w_cmd:.3f},"
                f"{left:.3f},{right:.3f},{s['fl']:.4f},{s['fr']:.4f},{s['sl']:.4f},{s['sr']:.4f},"
                f"{s['ticks_l']},{s['ticks_r']},{s['gyro'][2]:.4f},{g['x']:.4f},{g['y']:.4f}\n")
            self._n += 1
            if self._n % 100 == 0:
                self._log.flush()
        return left, right


def main():
    # The official maze, embedded - no file to find, no path to pass. It is
    # the same maze built into the simulator, and Maze.official() checks its
    # own copy has not been edited before handing it over.
    maze = Maze.official()
    print(f"official maze: {maze.grid}x{maze.grid}, "
          f"spawn row {maze.spawn_row}, exits {maze.exits}")

    controller = Controller(maze)
    client = _mqtt_client()

    def on_message(cli, userdata, msg):
        payload = json.loads(msg.payload.decode())
        if msg.topic == TOPIC_PELLETS:
            controller.on_pellets(payload)
            return
        if msg.topic == TOPIC_GHOST:
            controller.ghost = payload      # {"row": .., "col": .., "x": .., "y": ..}
            return
        if msg.topic == TOPIC_RESULT:
            print(f"run over: solved={payload['solved']}, "
                  f"collisions={payload['collisions']}")
            return
        left, right = controller.decide(payload)
        left = max(-WHEEL_LIMIT, min(WHEEL_LIMIT, float(left)))
        right = max(-WHEEL_LIMIT, min(WHEEL_LIMIT, float(right)))
        # Key order matters: the simulator matches a fixed pattern. See the
        # pacbot/wheel_vel notes in the header comment.
        cli.publish(TOPIC_WHEEL_VEL,
                    json.dumps({"left": left, "right": right}))

    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    client.subscribe(TOPIC_PELLETS)
    client.subscribe(TOPIC_GHOST)
    client.subscribe(TOPIC_RESULT)
    client.subscribe(TOPIC_SENSORS)
    print("connected; waiting for the simulator")
    client.loop_forever()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
