#!/usr/bin/env python3
"""
Number the parts of the handheld's main screen for the user manual.

    python3 docs/pdf/annotate.py      # docs/manual/img/03_main_patrol.png -> 03_main_tour.png

The screenshots come from the PashupatiController repository (test/screenshots_test.dart, the
`screenshots` workflow publishes them on its `screenshots` branch), 1668 x 938 px.
"""

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

DOCS = Path(__file__).resolve().parents[1]
FONT = DOCS / 'pdf' / 'fonts' / 'BarlowSemiCondensed-Bold.ttf'

# number: (x, y) of its badge on the screenshot, next to the part it names (manual chapter 4)
BADGES = {
    1: (112, 118),     # disconnect
    2: (560, 30),      # robot, link quality, latency
    3: (1075, 100),    # AUTO / REMOTE
    4: (1188, 125),    # localization
    5: (1314, 125),    # protective field
    6: (1455, 125),    # battery
    7: (1602, 125),    # settings
    8: (1480, 262),    # STOP
    9: (1470, 360),    # robot actions
    10: (440, 160),    # small view (camera)
    11: (480, 468),    # view buttons
    12: (390, 560),    # at a glance: AI, queue, people, load
    13: (790, 158),    # patrol status
    14: (1290, 470),   # route, stop point, robot
    15: (282, 862),    # toolbar: teach, patrol, inspect
}


def main():
    """Draw the numbered badges."""
    image = Image.open(DOCS / 'manual' / 'img' / '03_main_patrol.png').convert('RGB')
    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype(str(FONT), 30)
    radius = 23
    for number, (x, y) in BADGES.items():
        draw.ellipse(
            (x - radius - 3, y - radius - 3, x + radius + 3, y + radius + 3), fill='#ffffff'
        )
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill='#e8892c')
        draw.text((x, y + 1), str(number), fill='#ffffff', font=font, anchor='mm')
    image.save(DOCS / 'manual' / 'img' / '03_main_tour.png', optimize=True)
    print('docs/manual/img/03_main_tour.png')


if __name__ == '__main__':
    main()
