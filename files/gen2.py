"""Накладки-рамки для Systeme Electric AtlasDesign (механизм ATN000143), ставятся вместо рамки ATN000101.

Оси: X вправо, Y вверх, Z от стены. Мм. Печать лицом вверх (тыл на столе).
Посадка (замер по ортофото производителя, точность ±0.3–0.5 мм — сверить штангенциркулем):
  пластина механизма 71×71, крышка 56.2×56.2, окно рамки 56.8, 8 лапок под край крышки.
"""
import math, sys, json
import numpy as np
import manifold3d as m3d
import trimesh
from PIL import Image, ImageDraw
from scipy import ndimage
from shapely.geometry import Polygon, Point, LineString, box, MultiPolygon
from shapely import affinity
from shapely.ops import unary_union

FIT = dict(
    window=56.8, win_r=1.2,        # окно = крышка 56.2 + 0.3 на сторону
    cover_top=10.0,                # лицо крышки над стеной (сверить)
    lip=8.5,                       # высота рамки у окна (крышка выступает на 1.5 мм)
    plate=71.6, plate_r=2.0,       # карман под пластину 71 + 0.3 на сторону
    plate_z=1.1,                   # толщина пластины + 0.1
    rail_band=3.0, rail_z=2.3,     # рёбра по краю пластины
    tab_w=9.0, tab_c=11.0, tab_in=3.0, tab_t=1.2,  # лапки: ширина, центры ±11, заход под крышку 3.0, толщина
)
STEP = 0.3
LAYER_PATTERN = 0.4                # рельеф второго цвета: 2 слоя по 0.2


# ---------------- геометрия ----------------
def man_from_tm(tm):
    return m3d.Manifold(m3d.Mesh(vert_properties=np.asarray(tm.vertices, np.float32),
                                 tri_verts=np.asarray(tm.faces, np.uint32)))


def tm_from_man(m):
    mesh = m.to_mesh()
    return trimesh.Trimesh(mesh.vert_properties[:, :3], mesh.tri_verts, process=False)


def cs_from_shape(g):
    polys = [g] if isinstance(g, Polygon) else list(g.geoms)
    contours = []
    for p in polys:
        contours.append(np.asarray(p.exterior.coords[:-1], np.float64))
        for h in p.interiors:
            contours.append(np.asarray(h.coords[:-1], np.float64))
    return m3d.CrossSection(contours, m3d.FillRule.EvenOdd)


def prism(g, z0, z1):
    return cs_from_shape(g).extrude(z1 - z0).translate([0, 0, z0])


def rrect(w, h, r, cx=0, cy=0):
    return box(cx - w / 2 + r, cy - h / 2 + r, cx + w / 2 - r, cy + h / 2 - r).buffer(r, resolution=16)


def heightblock(H, xmin, ymin, step):
    """Замкнутое тело: верх z=H[j,i], низ z=0."""
    ny, nx = H.shape
    xs = xmin + np.arange(nx) * step
    ys = ymin + np.arange(ny) * step
    X, Y = np.meshgrid(xs, ys)
    top = np.stack([X.ravel(), Y.ravel(), H.ravel()], 1)
    idx = lambda i, j: j * nx + i
    I, J = np.meshgrid(np.arange(nx - 1), np.arange(ny - 1))
    a, b, c, d = idx(I, J), idx(I + 1, J), idx(I + 1, J + 1), idx(I, J + 1)
    faces = [np.stack([a, b, c], -1).reshape(-1, 3), np.stack([a, c, d], -1).reshape(-1, 3)]
    # периметр против часовой (вид сверху)
    per = [(i, 0) for i in range(nx)] + [(nx - 1, j) for j in range(1, ny)] + \
          [(i, ny - 1) for i in range(nx - 2, -1, -1)] + [(0, j) for j in range(ny - 2, 0, -1)]
    T = np.array([idx(i, j) for i, j in per])
    base = len(top)
    bot = np.array([[xs[i], ys[j], 0.0] for i, j in per])
    B = base + np.arange(len(per))
    cidx = base + len(per)
    verts = np.vstack([top, bot, [[xs.mean(), ys.mean(), 0.0]]])
    n = len(per)
    k = np.arange(n); k1 = (k + 1) % n
    faces.append(np.stack([B[k], B[k1], T[k1]], 1))
    faces.append(np.stack([B[k], T[k1], T[k]], 1))
    faces.append(np.stack([np.full(n, cidx), B[k1], B[k]], 1))
    tm = trimesh.Trimesh(verts, np.vstack(faces), process=False)
    return man_from_tm(tm)


class Grid:
    def __init__(self, half, step=STEP):
        self.step = step
        self.xmin = self.ymin = -half
        n = int(round(2 * half / step)) + 1
        xs = self.xmin + np.arange(n) * step
        self.X, self.Y = np.meshgrid(xs, xs)
        self.n = n

    def mask(self, g):
        img = Image.new("L", (self.n, self.n), 0)
        dr = ImageDraw.Draw(img)
        polys = [g] if isinstance(g, Polygon) else list(getattr(g, "geoms", []))
        tr = lambda cs: [((x - self.xmin) / self.step, (y - self.ymin) / self.step) for x, y in cs]
        for p in polys:
            dr.polygon(tr(p.exterior.coords), fill=1)
            for h in p.interiors:
                dr.polygon(tr(h.coords), fill=0)
        return np.asarray(img, bool)

    def dist_inside(self, g):
        """расстояние (мм) от точки внутри фигуры до её края"""
        return ndimage.distance_transform_edt(self.mask(g)) * self.step

    def dist_to(self, g):
        """расстояние (мм) до фигуры (0 внутри)"""
        return ndimage.distance_transform_edt(~self.mask(g)) * self.step


def window_shape():
    return rrect(FIT["window"], FIT["window"], FIT["win_r"])


def fit_cut(body):
    """вырезать окно, карман пластины, добавить 8 лапок"""
    W = FIT["window"]
    body = body - prism(window_shape(), -1, 60)
    P = FIT["plate"]
    body = body - prism(rrect(P, P, FIT["plate_r"]), -1, FIT["plate_z"])
    rb = rrect(P, P, FIT["plate_r"]).difference(rrect(P - 2 * FIT["rail_band"], P - 2 * FIT["rail_band"], 1))
    body = body - prism(rb, -1, FIT["rail_z"])
    tabs = []
    for side in range(4):
        for c in (-FIT["tab_c"], FIT["tab_c"]):
            t = box(c - FIT["tab_w"] / 2, W / 2 - FIT["tab_in"], c + FIT["tab_w"] / 2, W / 2 + 0.6)
            t = affinity.rotate(t, 90 * side, origin=(0, 0))
            tabs.append(t)
    body = body + prism(unary_union(tabs), FIT["plate_z"], FIT["plate_z"] + FIT["tab_t"])
    return body


def edge_round(d, r, hmax):
    """скругление края: d — расстояние от края контура"""
    t = np.clip(d / r, 0, 1)
    return hmax * np.sqrt(1 - (1 - t) ** 2)


def finish(outline, H, grid, top_extra=5):
    H = np.maximum(H, 0.6)
    blk = heightblock(H.astype(np.float64), grid.xmin, grid.ymin, grid.step)
    body = blk ^ prism(outline, -1, float(H.max()) + top_extra)
    return fit_cut(body)


def lip_band(grid, H, width=2.5):
    """у окна высота не ниже FIT.lip — рамка ровно обнимает крышку"""
    dw = grid.dist_to(window_shape())
    H = np.where(dw < width, np.maximum(H, FIT["lip"]), H)
    return H, dw


# ---------------- дизайны ----------------
def d_plain():
    g = Grid(50)
    outline = rrect(92, 92, 10)
    do = g.dist_inside(outline)
    H = np.minimum(edge_round(do, 6, FIT["lip"]), FIT["lip"])
    H, _ = lip_band(g, H)
    return finish(outline, H, g), {}


def d_spikes():
    g = Grid(56)
    outline = rrect(104, 104, 24)
    do = g.dist_inside(outline)
    H = edge_round(do, 5, 7.5)
    H, dw = lip_band(g, H)
    for ring, n, h, rb in ((46.5, 34, 6.0, 2.8), (41, 30, 5.2, 2.6), (35.5, 26, 4.2, 2.3)):
        for i in range(n):
            a = 2 * math.pi * (i + 0.5 * (ring % 2)) / n
            # шипы по скруглённому квадрату
            x, y = ring * math.cos(a), ring * math.sin(a)
            s = ring / max(abs(x), abs(y)) * 0.92 if max(abs(x), abs(y)) > 0 else 1
            x, y = x * s, y * s
            if max(abs(x), abs(y)) < FIT["window"] / 2 + 4:
                continue
            r = np.hypot(g.X - x, g.Y - y)
            cone = 7.5 + h * np.clip(1 - r / rb, 0, 1)
            cone = np.minimum(cone, 7.5 + h * (1 - 0.45 / rb))  # тупая вершина Ø0.9
            H = np.where(r < rb, np.maximum(H, cone), H)
    H = np.where(do < 2.5, np.minimum(H, edge_round(do, 5, 7.5)), H)
    return finish(outline, H, g), {}


def d_organic():
    g = Grid(62)
    th = np.linspace(0, 2 * np.pi, 400, endpoint=False)
    base = 54
    r = base * (1 + 0.055 * np.sin(5 * th + 0.4) + 0.035 * np.sin(3 * th + 1.7) + 0.02 * np.sin(8 * th))
    sq = 1 / np.maximum(np.abs(np.cos(th)), np.abs(np.sin(th))) ** 0.35  # чуть квадратнее
    r = r * sq / sq.max() * 1.08
    outline = Polygon(np.stack([r * np.cos(th), r * np.sin(th)], 1)).buffer(0)
    do = g.dist_inside(outline)
    dw = g.dist_to(window_shape())
    t = dw / np.maximum(dw + do, 1e-6)
    ang = np.arctan2(g.Y, g.X)
    roll = np.sin(np.pi * np.clip(t, 0, 1)) ** 0.7
    fold = np.sin(2 * np.pi * 2.2 * t + 1.3 * np.sin(3 * ang) + 0.7 * np.sin(5 * ang + 1))
    H = FIT["lip"] + 3.5 * roll + 1.4 * fold * roll
    H = np.minimum(H, edge_round(do, 7, 14))
    H = ndimage.gaussian_filter(H, 1.2 / g.step)
    H, _ = lip_band(g, H, 1.5)
    return finish(outline, H, g), {}


def ridge(g, line_geom, width, height):
    d = g.dist_to(line_geom.buffer(0.01))
    return height * np.clip(1 - (d / width) ** 2, 0, 1)


def d_baroque():
    g = Grid(56)
    outline = rrect(104, 104, 6)
    do = g.dist_inside(outline)
    base = 7.0
    # профиль багета: скругление + выпуклый валик у края
    H = edge_round(do, 4, base) + 1.6 * np.exp(-((do - 6.5) / 2.0) ** 2)
    H, dw = lip_band(g, H, 1.2)
    # жемчуг у окна
    half = FIT["window"] / 2 + 3.0
    pitch = 4.4
    for side in range(4):
        for k in np.arange(-half, half + 0.01, pitch):
            x, y = k, half
            x, y = [(x, y), (-y, x), (-x, -y), (y, -x)][side]
            r = np.hypot(g.X - x, g.Y - y)
            H = np.maximum(H, np.where(r < 1.7, base + np.sqrt(np.clip(1.7 ** 2 - r ** 2, 0, None)), 0))
    # розетки в углах
    for cx, cy in ((40, 40), (-40, 40), (-40, -40), (40, -40)):
        for i in range(8):
            a = 2 * math.pi * i / 8 + math.pi / 8
            px, py = cx + 5.2 * math.cos(a), cy + 5.2 * math.sin(a)
            u = ((g.X - px) * math.cos(a) + (g.Y - py) * math.sin(a)) / 5.0
            v = (-(g.X - px) * math.sin(a) + (g.Y - py) * math.cos(a)) / 2.6
            H = np.maximum(H, base + 3.0 * np.sqrt(np.clip(1 - u ** 2 - v ** 2, 0, None)))
        r = np.hypot(g.X - cx, g.Y - cy)
        H = np.maximum(H, base + 4.2 * np.sqrt(np.clip(1 - (r / 2.8) ** 2, 0, None)))
    # завитки (C-scroll) по серединам сторон
    for side in range(4):
        for sgn in (-1, 1):
            t = np.linspace(0, 1.75 * np.pi, 80)
            rr = 7.5 - 2.0 * t
            pts = np.stack([sgn * (11 + rr * np.cos(t)), 40.5 + rr * np.sin(t) * 0.8], 1)
            ln = LineString(pts)
            ln = affinity.rotate(ln, 90 * side, origin=(0, 0))
            H = np.maximum(H, base + ridge(g, ln, 1.3, 2.2))
            leaf = LineString([(sgn * 3, 40.5), (sgn * 11, 46), (sgn * 20, 44.5), (sgn * 26, 38)])
            leaf = affinity.rotate(leaf, 90 * side, origin=(0, 0))
            H = np.maximum(H, base + ridge(g, leaf, 1.1, 1.6))
        c = affinity.rotate(Point(0, 41), 90 * side, origin=(0, 0))
        r = np.hypot(g.X - c.x, g.Y - c.y)
        H = np.maximum(H, base + 3.0 * np.sqrt(np.clip(1 - (r / 2.4) ** 2, 0, None)))
    H = np.where(do < 4, np.minimum(H, edge_round(do, 4, base + 1.6)), H)
    H = ndimage.gaussian_filter(H, 0.35 / g.step)
    H, _ = lip_band(g, H, 1.2)
    return finish(outline, H, g), {}


def tile_base(size=104):
    g = Grid(size / 2 + 2)
    outline = rrect(size, size, 4)
    do = g.dist_inside(outline)
    H = np.minimum(edge_round(do, 3.5, FIT["lip"]), FIT["lip"])
    H, _ = lip_band(g, H)
    return g, outline, H


def flower(cx, cy, r, n=5, rot=0.0, petal_w=0.55):
    petals = []
    for i in range(n):
        a = rot + 2 * math.pi * i / n
        e = affinity.scale(Point(0, 0).buffer(1, 24), r * 0.55, r * 0.55 * petal_w * 1.6)
        e = affinity.rotate(e, math.degrees(a), origin=(0, 0))
        e = affinity.translate(e, cx + r * 0.55 * math.cos(a), cy + r * 0.55 * math.sin(a))
        petals.append(e)
    return unary_union(petals).difference(Point(cx, cy).buffer(r * 0.22)), Point(cx, cy).buffer(r * 0.18)


def leaf(cx, cy, ln, w, ang):
    e = affinity.scale(Point(0, 0).buffer(1, 24), ln / 2, w / 2)
    e = affinity.rotate(e, ang, origin=(0, 0))
    return affinity.translate(e, cx, cy)


def band_ok(geom, size=104):
    keep = rrect(size - 9, size - 9, 2).difference(rrect(FIT["window"] + 4, FIT["window"] + 4, 2))
    return geom.intersection(keep)


def d_delft():
    """синяя роспись: лоза с листьями и цветами, 2 цвета (смена прутка на слое)"""
    g, outline, H = tile_base()
    parts = []
    R = 40.5
    # лоза — волнистая линия по квадратному пути
    path = []
    L = 4 * 2 * R
    for s in np.linspace(0, L, 900, endpoint=False):
        side, u = int(s // (2 * R)), s % (2 * R) - R
        x, y = [(u, R), (R, -u), (-u, -R), (-R, u)][side]
        nx, ny = [(0, 1), (1, 0), (0, -1), (-1, 0)][side]
        w = 2.6 * math.sin(s / (2 * R) * 2 * math.pi * 3)
        path.append((x + nx * w, y + ny * w))
    vine = LineString(path + [path[0]]).buffer(0.5)
    parts.append(vine)
    for k in range(24):
        s = (k + 0.5) / 24 * L
        side, u = int(s // (2 * R)), s % (2 * R) - R
        x, y = [(u, R), (R, -u), (-u, -R), (-R, u)][side]
        nx, ny = [(0, 1), (1, 0), (0, -1), (-1, 0)][side]
        sg = 1 if k % 2 else -1
        ang = math.degrees(math.atan2(ny, nx)) + sg * 35 + (0 if side % 2 == 0 else 90) * 0
        lx, ly = x + nx * sg * 4.5, y + ny * sg * 4.5
        parts.append(leaf(lx + (ny * 1.5), ly - (nx * 1.5), 6.5, 2.4, ang + 90))
    for cx, cy in ((R, R), (-R, R), (-R, -R), (R, -R), (0, R), (R, 0), (0, -R), (-R, 0)):
        f, c = flower(cx, cy, 6.0, 6)
        parts += [f, c]
    parts.append(rrect(98, 98, 3).difference(rrect(96.2, 96.2, 2.2)))  # тонкая кайма
    pat = band_ok(unary_union(parts).buffer(0.05)).buffer(0)
    body = finish(outline, H, g)
    p1 = prism(pat, FIT["lip"] - 0.6, FIT["lip"] + LAYER_PATTERN)
    return body - prism(pat, FIT["lip"] - 0.6, FIT["lip"] + 1), {"p1": p1}


def d_tile():
    """плиточный узор: зелёные листья (слой 1) и терракотовые цветы (слой 2)"""
    g, outline, H = tile_base()
    flowers, leaves = [], []
    pts = [(-38, 38), (-19, 41), (0, 38), (19, 41), (38, 38), (41, 19), (38, 0), (41, -19),
           (38, -38), (19, -41), (0, -38), (-19, -41), (-38, -38), (-41, -19), (-38, 0), (-41, 19)]
    for i, (x, y) in enumerate(pts):
        big = i % 2 == 0
        f, c = flower(x, y, 6.2 if big else 4.4, 5, rot=i * 0.4)
        flowers += [f, c]
        a = math.degrees(math.atan2(y, x))
        for s in (-1, 1):
            r0 = 8.6 if big else 6.6
            leaves.append(leaf(x + r0 * math.cos(math.radians(a + 90 * s)), y + r0 * math.sin(math.radians(a + 90 * s)), 6.0, 2.4, a + 90 * s + 25 * s))
    leaves.append(rrect(98, 98, 3).difference(rrect(96.2, 96.2, 2.2)))
    fl = band_ok(unary_union(flowers)).buffer(0)
    lv = band_ok(unary_union(leaves)).difference(fl.buffer(0.6)).buffer(0)
    body = finish(outline, H, g)
    z0 = FIT["lip"]
    p1 = prism(lv, z0 - 0.6, z0 + LAYER_PATTERN)
    p2 = prism(fl, z0 - 0.6, z0 + 2 * LAYER_PATTERN)
    body = body - prism(unary_union([fl, lv]), z0 - 0.6, z0 + 2)
    return body, {"p1": p1, "p2": p2}


def d_cat():
    """крутой кот в очках: голова + уши, очки вторым цветом (выше слоя 11.2)"""
    g = Grid(86)
    head = affinity.scale(Point(0, 6).buffer(1, 64), 62, 57)
    ears = []
    for s in (-1, 1):
        ears.append(Polygon([(s * 22, 52), (s * 55, 30), (s * 50, 76)]).buffer(5, join_style=1))
    outline = unary_union([head] + ears).buffer(-2).buffer(2)
    do = g.dist_inside(outline)
    H = FIT["lip"] + 2.5 * np.clip(do / 22, 0, 1) ** 0.6
    H = np.minimum(H, edge_round(do, 6, 11.0))
    # внутренняя часть уха — углубление
    for s in (-1, 1):
        inner = Polygon([(s * 30, 56), (s * 49, 41), (s * 47, 68)]).buffer(2, join_style=1)
        d = g.dist_inside(inner)
        H = np.where(d > 0, H - 1.2 * np.clip(d / 1.5, 0, 1), H)
    # усы — валики
    for s in (-1, 1):
        for k, (y0, y1) in enumerate(((6, 10), (0, 0), (-6, -11))):
            ln = LineString([(s * 33, y0), (s * 47, (y0 + y1) / 2 + 0.6), (s * 60, y1)])
            H = np.maximum(H, np.minimum(11.0, H + ridge(g, ln, 0.9, 1.0)))
    H = ndimage.gaussian_filter(H, 0.4 / g.step)
    H = np.minimum(H, 11.0)
    H, _ = lip_band(g, H)
    body = finish(outline, H, g)
    # очки
    lens = []
    for s in (-1, 1):
        l = rrect(28, 16, 6, s * 17.5, 43)
        l = affinity.rotate(l, -s * 6, origin=(s * 17.5, 43))
        lens.append(l)
    bridge = rrect(10, 3.2, 1.2, 0, 47)
    arms = [rrect(14, 3, 1.2, s * 37, 46) for s in (-1, 1)]
    glasses = unary_union(lens + [bridge] + arms).intersection(outline.buffer(-5))
    p1 = prism(glasses, 8.0, 14.6)
    shine = unary_union([affinity.rotate(rrect(9, 1.6, 0.8, s * 13, 47), -25, origin=(s * 13, 47)) for s in (-1, 1)])
    p1 = p1 - prism(shine, 14.0, 15)
    body = body - prism(glasses, 8.0, 20)
    return body, {"p1": p1}


DESIGNS = dict(plain=d_plain, spikes=d_spikes, organic=d_organic, baroque=d_baroque,
               delft=d_delft, tile=d_tile, cat=d_cat)


def export(name, out):
    body, extra = DESIGNS[name]()
    parts = {"body": body, **extra}
    info = {}
    scene = trimesh.Scene()
    for k, m in parts.items():
        tm = tm_from_man(m)
        assert tm.is_watertight, (name, k)
        tm.export(f"{out}/{name}_{k}.stl")
        info[k] = dict(tris=len(tm.faces), vol_cm3=round(tm.volume / 1000, 1),
                       bbox=[round(v, 1) for v in tm.extents])
        web = tm
        if len(tm.faces) > 160000:
            import fast_simplification
            v, f = fast_simplification.simplify(tm.vertices, tm.faces, target_reduction=1 - 160000 / len(tm.faces))
            web = trimesh.Trimesh(v, f)
        scene.add_geometry(web, node_name=k, geom_name=k)
    scene.export(f"{out}/{name}.glb")
    return info


if __name__ == "__main__":
    out = sys.argv[1]
    names = sys.argv[2:] or list(DESIGNS)
    res = {}
    for n in names:
        res[n] = export(n, out)
        print(n, json.dumps(res[n], ensure_ascii=False), flush=True)
