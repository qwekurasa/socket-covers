"""
Параметрическая декоративная накладка на розетку
Systeme Electric AtlasDesign ATN000143 + рамка ATN000101 (1 пост).

Накладка надевается поверх уже установленной розетки (рамка + лицевая панель),
держится трением о боковины рамки (+ опционально тонкий двусторонний скотч),
открытым остаётся только гнездо Schuko.

Все размеры в мм. ПЕРЕД ПЕЧАТЬЮ ЗАМЕРИТЬ ШТАНГЕНЦИРКУЛЕМ и поправить FIT.
"""
import math
import sys
from build123d import *

# ---------- ПОСАДКА (зависит от модели розетки) ----------
FIT = dict(
    frame_w=83.0,      # внешний размер рамки ATN000101 (паспорт: 83x83)
    frame_r=4.0,       # радиус скругления углов рамки (замерить)
    frame_h=12.0,      # высота рамки+панели над стеной в самой высокой точке (паспорт рамки: 12)
    clr=0.25,          # зазор на сторону
    hole_d=40.0,       # диаметр гнезда Schuko по верхней кромке (замерить; ставим +0.5)
    rib=0.35,          # выступ рёбер-«зацепов» внутри юбки для посадки на трение
)

# ---------- ОБЩИЕ ПАРАМЕТРЫ НАКЛАДКИ ----------
WALL = 2.4            # толщина юбки
FACE = 1.6            # толщина лицевой стенки (минимум, чтобы вилка входила почти как раньше)
HOLE_CHAMFER = 0.8    # фаска на кромке отверстия
KEEP_OUT = 5.0        # чистая зона вокруг отверстия без декора (под корпус вилки)

TOP = FIT["frame_h"] + FACE          # z лицевой поверхности
HOLE_R = FIT["hole_d"] / 2


def pocket_and_hole(body):
    """Вырезать карман под рамку, отверстие, добавить рёбра посадки."""
    pw = FIT["frame_w"] + 2 * FIT["clr"]
    with BuildPart() as cut:
        with BuildSketch():
            RectangleRounded(pw, pw, FIT["frame_r"] + FIT["clr"])
        extrude(amount=FIT["frame_h"])
    body = body - cut.part
    # отверстие
    hole = Cylinder(HOLE_R, 200, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    body = body - hole
    # фаска на кромке отверстия (конус)
    c = HOLE_CHAMFER
    cone = Cone(HOLE_R, HOLE_R + c, c, align=(Align.CENTER, Align.CENTER, Align.MIN))
    cone = cone.moved(Location((0, 0, TOP - c + 0.001)))
    body = body - cone
    # рёбра посадки: по 3 вертикальных полуцилиндра на сторону
    ribs = []
    half = pw / 2
    for side in range(4):
        for t in (-22, 0, 22):
            r = Cylinder(1.0, FIT["frame_h"] - 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
            r = r.moved(Location((t, half + 1.0 - FIT["rib"], 1)))
            r = r.rotate(Axis.Z, 90 * side)
            ribs.append(r)
    rib_union = Part() + ribs
    # оставить только часть рёбер внутри кармана
    with BuildPart() as ring:
        with BuildSketch():
            RectangleRounded(pw + 4, pw + 4, FIT["frame_r"] + 2)
            RectangleRounded(pw - 4, pw - 4, max(FIT["frame_r"] - 2, 0.5), mode=Mode.SUBTRACT)
        extrude(amount=FIT["frame_h"])
    body = body + (rib_union & ring.part)
    return body


SKIRT_S = FIT["frame_w"] + 2 * (FIT["clr"] + WALL)
SKIRT_R = FIT["frame_r"] + FIT["clr"] + WALL


def base_from_outline(outline_sketch_fn):
    """Тело = декоративный контур ∪ обязательная юбка вокруг рамки.
    Всё, что выходит за юбку, облегчается снизу (остаётся бортик WALL по контуру)."""
    with BuildSketch() as sk:
        outline_sketch_fn()
        RectangleRounded(SKIRT_S, SKIRT_S, SKIRT_R)
    outline = sk.sketch
    body = extrude(outline, amount=TOP)
    # облегчение: внутренний контур минус зона юбки
    inner = offset(outline, amount=-WALL)
    skirt = Rectangle(SKIRT_S + 2 * WALL, SKIRT_S + 2 * WALL)
    skirt = RectangleRounded(SKIRT_S + 2 * WALL, SKIRT_S + 2 * WALL, SKIRT_R + WALL)
    hollow = inner - skirt
    if hollow.area > 1:
        body = body - extrude(hollow, amount=FIT["frame_h"])
    return body


def keep_out_cyl(h=50):
    return Cylinder(HOLE_R + KEEP_OUT, h, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, TOP - 0.01)))


# ================== ВАРИАНТЫ ДИЗАЙНА ==================

def v_plain():
    S = FIT["frame_w"] + 2 * (FIT["clr"] + WALL)
    body = base_from_outline(lambda: RectangleRounded(S, S, 8))
    body = body.fillet(1.2, body.edges().group_by(Axis.Z)[-1]) if False else body
    return pocket_and_hole(body)


def v_spikes():
    """Ёжик: овал-круг с коническими шипами (тупые вершины, печать лицом вверх)."""
    body = base_from_outline(lambda: RectangleRounded(100, 100, 24))
    cones = []
    for ring_r, n, h, rb in ((52.0, 34, 6.0, 2.6), (46.0, 30, 5.0, 2.4), (40.0, 26, 3.8, 2.2), (34.0, 22, 2.8, 1.9), (28.5, 18, 2.0, 1.6)):
        for i in range(n):
            a = 2 * math.pi * i / n + (math.pi / n if n % 4 else 0)
            # шипы только внутри контура
            if ring_r > 44 and abs(math.cos(a)) * ring_r > 46 and abs(math.sin(a)) * ring_r > 46:
                continue
            co = Cone(rb, 0.45, h, align=(Align.CENTER, Align.CENTER, Align.MIN))
            cones.append(co.moved(Location((ring_r * math.cos(a), ring_r * math.sin(a), TOP - 0.01))))
    body = body + cones
    body = body - keep_out_cyl()
    return pocket_and_hole(body)


def v_waves():
    """Волны: скруглённый квадрат с рельефными синусоидальными полосами."""
    S = 96
    body = base_from_outline(lambda: RectangleRounded(S, S, 10))
    bands = []
    w = 2.4
    for k in range(-7, 8):
        x0 = k * 6.5
        pts_l, pts_r = [], []
        for j in range(61):
            y = -S / 2 + S * j / 60
            x = x0 + 2.2 * math.sin(y / 7.0 + k * 0.9)
            pts_l.append((x - w / 2, y))
            pts_r.append((x + w / 2, y))
        with BuildPart() as b:
            with BuildSketch(Plane.XY.offset(TOP - 0.01)):
                Polygon(*(pts_l + pts_r[::-1]))
            extrude(amount=1.2)
        bands.append(b.part)
    with BuildPart() as clip:
        with BuildSketch(Plane.XY.offset(TOP - 0.5)):
            RectangleRounded(S - 3, S - 3, 8.5)
        extrude(amount=5)
    deco = (Part() + bands) & clip.part
    body = body + deco
    body = body - keep_out_cyl()
    return pocket_and_hole(body)


def v_flower():
    """Цветок: волнистый контур из 8 лепестков + радиальные рёбра."""
    def outline():
        Circle(48)
        for i in range(8):
            a = 2 * math.pi * i / 8
            with Locations((47 * math.cos(a), 47 * math.sin(a))):
                Circle(15)
    body = base_from_outline(outline)
    ridges = []
    for i in range(16):
        a = 360 * i / 16
        with BuildPart() as r:
            with BuildSketch(Plane.XY.offset(TOP - 0.01)):
                with Locations((HOLE_R + KEEP_OUT + 11, 0)):
                    SlotCenterToCenter(18, 2.6)
            extrude(amount=1.4)
        ridges.append(r.part.rotate(Axis.Z, a))
    # бусины на лепестках
    for i in range(8):
        a = 2 * math.pi * i / 8
        s = Cone(3.4, 2.2, 1.8, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((50 * math.cos(a), 50 * math.sin(a), TOP - 0.01)))
        ridges.append(s)
    body = body + ridges
    body = body - keep_out_cyl()
    return pocket_and_hole(body)


VARIANTS = dict(plain=v_plain, spikes=v_spikes, waves=v_waves, flower=v_flower)

if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    names = sys.argv[2:] or list(VARIANTS)
    for n in names:
        part = VARIANTS[n]()
        export_stl(part, f"{out}/socket_cover_{n}.stl", tolerance=0.02, angular_tolerance=0.15)
        export_step(part, f"{out}/socket_cover_{n}.step")
        bb = part.bounding_box()
        print(n, "valid" if part.is_valid else "INVALID", f"{bb.size.X:.1f}x{bb.size.Y:.1f}x{bb.size.Z:.1f} mm", f"vol {part.volume/1000:.1f} cm3")
