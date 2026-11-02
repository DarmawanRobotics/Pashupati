---
layout: home

hero:
  name: Pashupati
  text: Patrol and inspection, taught once
  tagline: >-
    Map a site, drive a route with its stop points, and the robot repeats it every shift: AI
    inspection at each stop, crowd sensing on the way, a handheld to take over, and every finding
    in the NETRA command center. Built on the Dharma stack.
  image:
    src: /logo.png
    alt: Pashupati
  actions:
    - theme: brand
      text: User Manual
      link: /guides/user-manual
    - theme: alt
      text: Tuning Guide
      link: /guides/tuning-guide
    - theme: alt
      text: Install
      link: /reference/overview#install-on-the-robot

features:
  - icon: 🗺️
    title: Teach and repeat
    details: >-
      Map a site once, record AprilTags and a route with stop points by driving it; the robot
      repeats it lap after lap, localized on the tags.
    link: /guides/user-manual
    linkText: Teaching a route
  - icon: 🔍
    title: AI inspection
    details: >-
      A vision-language model on the robot looks at every stop point for the finding types you
      define: trash, spills, blocked exits, open doors.
    link: /packages/pashupati_inspection
    linkText: Inspection
  - icon: 🎮
    title: Handheld in control
    details: >-
      The Pashupati Controller drives the robot by hand, switches AUTO and REMOTE, shows the
      video, the LiDAR map and the patrol, and stops it.
    link: /reference/handheld-protocol
    linkText: Handheld protocol
  - icon: 🛰️
    title: NETRA command center
    details: >-
      Live position and health, patrols on a schedule, findings with their photo, crowd maps and
      alerts, on one clock with the robot.
    link: /reference/netra-api
    linkText: NETRA API
  - icon: 🛡️
    title: Safe by construction
    details: >-
      One mux drives the robot: e-stop, a protective field from the measured braking, and a
      health signal that slows it down when a sensor lags.
    link: /guides/tuning-guide
    linkText: Protective field
  - icon: 🦿
    title: Any quadruped
    details: >-
      Legged or wheel-legged, any brand with a ROS 2 driver: a robot package and a robot profile
      are all it takes.
    link: /reference/robot-packages
    linkText: Robot packages
---

## Guides

<div class="docs-cards">

[**User Manual** · operators<br><span>Driving from the handheld, teaching routes, patrols, AI inspection, troubleshooting. No robotics knowledge needed.</span>](/guides/user-manual)

[**Tuning Guide** · technicians<br><span>What every parameter does, the site file, tuning a robot for a site, time synchronisation, all parameters.</span>](/guides/tuning-guide)

[**PDF for the phone**<br><span>Both guides as A5 PDFs, to keep on the phone during a shift: User Manual and Tuning Guide.</span>](/files/Pashupati-User-Manual.pdf)

</div>
