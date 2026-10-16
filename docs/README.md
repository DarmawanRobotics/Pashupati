# Documentation

| Guide | For | Open |
|---|---|---|
| **User Manual** | operators: driving from the handheld, teaching routes, patrols, AI inspection, troubleshooting. No robotics knowledge needed | [PDF](Pashupati-User-Manual.pdf) · [source](manual/user-manual.md) |
| **Tuning Guide** | technicians: what every parameter does, the site file, and how to tune a robot for a site, with a reference of all parameters | [PDF](Pashupati-Tuning-Guide.pdf) · [source](tuning/tuning-guide.md) |

Both are A5 PDFs, sized to read on a phone during a shift or a tuning session: open them here on
GitHub or keep them on the phone. They describe Pashupati on the ZSBot Genisom L1W with the
[Pashupati Controller](https://github.com/DarmawanRobotics/PashupatiController) handheld; the
parameter reference holds for every robot.

## Other documents

| Document | Covers |
|---|---|
| [README](../README.md) | install, configuration, workflow, operations, development |
| [Robot packages](../src/xtras/README.md) | what a robot package must provide, checklist for a new robot |
| [Handheld protocol](../src/robot_bridge/PROTOCOL.md) | the UDP protocol between the robot and the Pashupati Controller |
| [NETRA API](../src/robot_fleet/API.md) | what the robot sends to and takes from NETRA |
| Package READMEs (`src/robot_*/README.md`) | nodes, topics, services and parameters of each package |
| [CONTRIBUTING](../CONTRIBUTING.md) | branches, commits, rules that come from real crashes |
| [CHANGELOG](../CHANGELOG.md) | changes per version |

## Rebuilding the PDFs

Rebuild after changing a guide, or any `src/*/config/*.yaml` (the Tuning Guide reads its parameter
reference from them), and commit the PDFs with the change. CI builds them on every push and warns
when a committed PDF is older than its sources.

```bash
pip install weasyprint markdown pyyaml pillow   # macOS: brew install pango first
python3 docs/pdf/build.py                        # both guides
python3 docs/pdf/build.py tuning                 # one of them: manual or tuning
```

| Path | Holds |
|---|---|
| `manual/user-manual.md`, `tuning/tuning-guide.md` | the sources: Markdown with a YAML front matter (title, version, date, revisions) |
| `manual/img/` | screenshots of the handheld and the diagrams |
| `pdf/build.py` | Markdown to A5 PDF with WeasyPrint: cover, contents, numbered chapters, the parameter reference |
| `pdf/style.css` | page layout and colours |
| `pdf/fonts/` | Barlow Semi Condensed ([SIL Open Font License](pdf/fonts/OFL.txt)) |
| `pdf/diagrams.py` | draws `system.png` and `workflow.png` |
| `pdf/annotate.py` | numbers the parts of the main screen, `03_main_tour.png` |

For a new version of a guide, raise `version` and add a line to `revisions` in its front matter.
Chapters (`#`) and sections (`##`) are numbered in the PDF; refer to them as "section 3.3".

### Screenshots

The handheld screenshots come from the Pashupati Controller repository: its `screenshots`
workflow renders every screen with a simulated robot (`test/screenshots_test.dart`) and publishes
the images on its `screenshots` branch.

```bash
git clone --depth 1 --branch screenshots https://github.com/DarmawanRobotics/PashupatiController /tmp/shots
cp /tmp/shots/*.png docs/manual/img/
python3 docs/pdf/annotate.py && python3 docs/pdf/build.py manual
```
