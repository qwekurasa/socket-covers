"""Механизм Systeme Electric AtlasDesign (ATN0010x5) + рамка ATN000101. Z вверх, стена z=0, мм."""
import sys
from build123d import *

FRONT = 10.0          # лицо крышки над стеной (сверить)
BACK = -32.0          # 42 мм общая глубина
COVER = 56.2          # крышка (замер по ортофото)
REC_R = 19.5          # гнездо Schuko
REC_FLOOR = FRONT - 17.5
PIN_X = 9.5


def plate():
    with BuildPart() as p:
        with BuildSketch():
            RectangleRounded(71, 71, 2)
            Rectangle(46.5, 46.5, mode=Mode.SUBTRACT)
            with Locations((-31.6, 31.6), (31.6, 31.6), (-31.6, -31.6), (31.6, -31.6)):
                Circle(2.0, mode=Mode.SUBTRACT)
            with Locations((1.7, 30.3), (-1.7, -30.3)):
                SlotCenterToCenter(13, 4.0, mode=Mode.SUBTRACT)
            with Locations((-30.3, 1.7), (30.3, -1.7)):
                SlotCenterToCenter(13, 4.0, rotation=90, mode=Mode.SUBTRACT)
        extrude(amount=1.0)
        # отбортовка по краям между угловыми отверстиями
        for x in (-34.7, 34.7):
            with BuildSketch(Plane.XY.offset(1.0)):
                with Locations((x, 0)):
                    Rectangle(1.6, 50)
            extrude(amount=0.9)
        for y in (-34.7, 34.7):
            with BuildSketch(Plane.XY.offset(1.0)):
                with Locations((0, y)):
                    Rectangle(50, 1.6)
            extrude(amount=0.9)
    return p.part


def body():
    with BuildPart() as b:
        with BuildSketch(Plane.XY.offset(BACK)):
            RectangleRounded(45, 45, 2)
        extrude(amount=-BACK + 1.0)
        with BuildSketch(Plane.XY.offset(1.0)):
            RectangleRounded(COVER, COVER, 1.0)
        extrude(amount=FRONT - 1.0)
        chamfer(b.edges().group_by(Axis.Z)[-1], 0.5)
        # гнездо
        with BuildSketch(Plane.XY.offset(FRONT)):
            Circle(REC_R)
        extrude(amount=-(FRONT - REC_FLOOR), mode=Mode.SUBTRACT)
        fillet(b.edges().filter_by(GeomType.CIRCLE).group_by(Axis.Z)[-1], 0.8)
        # отверстия под штыри
        with BuildSketch(Plane.XY.offset(REC_FLOOR)):
            with Locations((-PIN_X, 0), (PIN_X, 0)):
                Circle(2.5)
        extrude(amount=-4, mode=Mode.SUBTRACT)
        # пазы под заземляющие клеммы в стенке
        for y in (REC_R, -REC_R):
            with BuildSketch(Plane.XY.offset(REC_FLOOR)):
                with Locations((0, y)):
                    Rectangle(7.5, 2.4)
            extrude(amount=FRONT - REC_FLOOR - 3, mode=Mode.SUBTRACT)
        # углубление под винт
        with BuildSketch(Plane.XY.offset(REC_FLOOR)):
            Circle(4.2)
        extrude(amount=-1.2, mode=Mode.SUBTRACT)
        # клеммы сзади: окна под провода
        with BuildSketch(Plane.XZ.offset(-22.5)):
            with Locations((-10, -22), (0, -22), (10, -22)):
                Rectangle(5, 4)
        extrude(amount=-4, mode=Mode.SUBTRACT)
    return b.part


def shutters():
    with BuildPart() as s:
        with BuildSketch(Plane.XY.offset(REC_FLOOR - 3.2)):
            with Locations((-PIN_X, 0), (PIN_X, 0)):
                Circle(2.6)
        extrude(amount=1.0)
    return s.part


def earth_clips():
    parts = []
    for sgn in (1, -1):
        with BuildPart() as c:
            y0 = sgn * (REC_R - 0.6)
            with BuildSketch(Plane.XY.offset(REC_FLOOR + 0.5)):
                with Locations((0, y0)):
                    Rectangle(6, 0.6)
            extrude(amount=FRONT - REC_FLOOR - 4)
            # выгиб-пружина внутрь
            with BuildSketch(Plane.XY.offset(REC_FLOOR + 5)):
                with Locations((0, y0 - sgn * 0.7)):
                    Rectangle(6, 1.0)
            extrude(amount=4)
            # загиб у пола
            with BuildSketch(Plane.XY.offset(REC_FLOOR + 0.2)):
                with Locations((0, y0 - sgn * 2)):
                    Rectangle(6, 4)
            extrude(amount=0.5)
        parts.append(c.part)
    return parts[0] + parts[1]


def screws():
    def head(x, y, z, r):
        with BuildPart() as h:
            with BuildSketch(Plane.XY.offset(z)):
                with Locations((x, y)):
                    Circle(r)
            extrude(amount=1.4)
            fillet(h.edges().group_by(Axis.Z)[-1], 0.5)
            for rot in (0, 90):
                with BuildSketch(Plane.XY.offset(z + 1.4)):
                    with Locations((x, y)):
                        Rectangle(r * 1.7, 0.7, rotation=rot)
                extrude(amount=-0.7, mode=Mode.SUBTRACT)
        return h.part
    return head(0, 0, REC_FLOOR - 1.2, 3.4)


def claws():
    parts = []
    for sgn in (-1, 1):
        with BuildPart() as c:
            with BuildSketch(Plane.XY.offset(-16)):
                with Locations((sgn * 26, 0)):
                    Rectangle(5, 16)
            extrude(amount=12)
            with BuildSketch(Plane.XY.offset(-16)):
                with Locations((sgn * 29.5, 0)):
                    Rectangle(5, 10)
            extrude(amount=3)
            with BuildSketch(Plane.XY.offset(-16)):
                with Locations((sgn * 31, 0)):
                    Circle(1.5)
            extrude(amount=17)
        parts.append(c.part)
    return parts[0] + parts[1]


def frame():
    with BuildPart() as f:
        with BuildSketch():
            RectangleRounded(83, 83, 4)
        extrude(amount=10)
        with BuildSketch():
            RectangleRounded(50.4, 50.4, 2.3)
        extrude(amount=10, mode=Mode.SUBTRACT)
        with BuildSketch():
            RectangleRounded(79, 79, 3)
        extrude(amount=8.5, mode=Mode.SUBTRACT)
    p = f.part
    try:
        outer = [e for e in p.edges().group_by(Axis.Z)[-1] if max(abs(e.center().X), abs(e.center().Y)) > 30]
        p = p.fillet(2.5, outer)
    except Exception as ex:
        print("frame fillet skipped", ex)
    return p


if __name__ == "__main__":
    out = sys.argv[1]
    parts = dict(plate=plate(), body=body(), shutters=shutters(), clips=earth_clips(),
                 screws=screws(), claws=claws())
    for n, p in parts.items():
        export_stl(p, f"{out}/mech_{n}.stl", tolerance=0.02, angular_tolerance=0.15)
        bb = p.bounding_box()
        print(n, p.is_valid, f"{bb.size.X:.1f}x{bb.size.Y:.1f}x{bb.size.Z:.1f}")
