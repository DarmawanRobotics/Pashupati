#!/usr/bin/env python3
"""
Draw the user manual's diagrams (docs/manual/img/system.png and workflow.png).

    python3 docs/pdf/diagrams.py
"""

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

DOCS = Path(__file__).resolve().parents[1]
FONTS = DOCS / 'pdf' / 'fonts'
INK, MUTED, LINE = '#16212a', '#5b6b75', '#c8d2d7'
TEAL, TEAL_SOFT, ORANGE, ORANGE_SOFT = '#1f9e93', '#e3f4f2', '#e8892c', '#fdf0e2'


def font(weight: str, size: int):
    """Return Barlow Semi Condensed in a weight (Regular, Medium, SemiBold, Bold)."""
    return ImageFont.truetype(str(FONTS / f'BarlowSemiCondensed-{weight}.ttf'), size)


def box(draw, xy, title, lines=(), fill='#ffffff', outline=LINE, title_fill=INK, width=3):
    """Draw a rounded box with a bold title and smaller lines under it."""
    draw.rounded_rectangle(xy, radius=22, fill=fill, outline=outline, width=width)
    x0, y0, x1, _ = xy
    cx = (x0 + x1) / 2
    draw.text((cx, y0 + 34), title, fill=title_fill, font=font('Bold', 40), anchor='mm')
    for i, line in enumerate(lines):
        draw.text((cx, y0 + 82 + i * 38), line, fill=MUTED, font=font('Medium', 31), anchor='mm')


def arrow(draw, start, end, label='', color=TEAL, both=False, label_above=True):
    """Draw a straight arrow, with heads on both ends when both is True, and its label."""
    draw.line([start, end], fill=color, width=7)
    for tip, tail in ([(end, start), (start, end)] if both else [(end, start)]):
        dx, dy = tip[0] - tail[0], tip[1] - tail[1]
        length = (dx * dx + dy * dy) ** 0.5
        ux, uy = dx / length, dy / length
        left = (tip[0] - 30 * ux + 16 * uy, tip[1] - 30 * uy - 16 * ux)
        right = (tip[0] - 30 * ux - 16 * uy, tip[1] - 30 * uy + 16 * ux)
        draw.polygon([tip, left, right], fill=color)
    if label:
        mx, my = (start[0] + end[0]) / 2, (start[1] + end[1]) / 2
        for i, line in enumerate(label.split('\n')):
            y = my - 44 + i * 34 if label_above else my + 30 + i * 34
            draw.text((mx, y), line, fill=color, font=font('SemiBold', 30), anchor='mm')


def system():
    """Draw the parts: handheld, robot (sensors, computer, motion), NETRA, Telegram."""
    image = Image.new('RGB', (2300, 970), '#ffffff')
    draw = ImageDraw.Draw(image)
    # robot: sensors feed the computer, the computer drives the motion computer
    draw.rounded_rectangle((650, 30, 1650, 930), radius=34, fill='#f6f9fa', outline=TEAL, width=5)
    draw.text((1150, 88), 'Robot: ZSBot Genisom L1W', fill=TEAL, font=font('Bold', 46),
              anchor='mm')
    lidar = ['Livox MID-360:', 'where the robot is,', 'obstacles']
    box(draw, (710, 140, 1130, 340), 'LiDAR', lidar)
    box(draw, (1170, 140, 1590, 340), 'Camera', ['RealSense D435i:', 'video, AprilTags,', 'AI'])
    box(draw, (710, 430, 1590, 640), 'Robot computer (Jetson Orin NX)',
        ['Pashupati: route following, safety,', 'AI inspection, people counting,',
         'handheld link, NETRA uplink'], fill=TEAL_SOFT, outline=TEAL)
    box(draw, (710, 730, 1590, 860), 'Motion computer (ZSBot)',
        ['walks, balances, stands and sits'])
    arrow(draw, (920, 342), (920, 428))
    arrow(draw, (1380, 342), (1380, 428))
    arrow(draw, (1150, 642), (1150, 728), both=True)
    # the handheld and NETRA talk to the robot computer
    box(draw, (30, 435, 500, 635), 'Pashupati Controller',
        ['Retroid Pocket handheld:', 'drive, watch, teach,', 'patrol, inspection'],
        fill=ORANGE_SOFT, outline=ORANGE)
    arrow(draw, (502, 535), (708, 535), color=ORANGE, both=True)
    draw.text((576, 492), 'Wi-Fi', fill=ORANGE, font=font('SemiBold', 32), anchor='mm')
    box(draw, (1800, 435, 2270, 635), 'NETRA',
        ['command center:', 'schedules, findings,', 'live map, reports'])
    arrow(draw, (1592, 535), (1798, 535), both=True)
    draw.text((1725, 492), 'network', fill=TEAL, font=font('SemiBold', 32), anchor='mm')
    box(draw, (1800, 730, 2270, 860), 'Telegram', ['alerts to the team'])
    arrow(draw, (2035, 637), (2035, 728))
    image.save(DOCS / 'manual' / 'img' / 'system.png', optimize=True)


def workflow():
    """Draw the operator's steps: teach once, then patrol."""
    steps = [
        ('1', 'Teach', ['drive the route,', 'mark stops,', 'record tags']),
        ('2', 'Localize', ['the robot finds', 'itself from', 'an AprilTag']),
        ('3', 'Load', ['pick the route', 'in Routes']),
        ('4', 'Patrol', ['AUTO, Start:', 'follows, stops,', 'inspects, loops']),
        ('5', 'Watch', ['findings on the', 'handheld and', 'in NETRA']),
    ]
    image = Image.new('RGB', (2160, 560), '#ffffff')
    draw = ImageDraw.Draw(image)
    width, gap = 360, 70
    for i, (number, title, lines) in enumerate(steps):
        x0 = 40 + i * (width + gap)
        fill, outline = (ORANGE_SOFT, ORANGE) if i == 0 else (TEAL_SOFT, TEAL)
        draw.rounded_rectangle((x0, 110, x0 + width, 500), radius=26, fill=fill,
                               outline=outline, width=4)
        cx = x0 + width / 2
        draw.ellipse((cx - 46, 64, cx + 46, 156), fill=outline)
        draw.text((cx, 110), number, fill='#ffffff', font=font('Bold', 50), anchor='mm')
        draw.text((cx, 215), title, fill=INK, font=font('Bold', 52), anchor='mm')
        for j, line in enumerate(lines):
            draw.text((cx, 292 + j * 44), line, fill=MUTED, font=font('Medium', 36), anchor='mm')
        if i < len(steps) - 1:
            arrow(draw, (x0 + width + 8, 305), (x0 + width + gap - 8, 305))
    image.save(DOCS / 'manual' / 'img' / 'workflow.png', optimize=True)


if __name__ == '__main__':
    system()
    workflow()
    print('docs/manual/img/system.png, docs/manual/img/workflow.png')
