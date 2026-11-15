#!/usr/bin/env bash
# The Dharma console of this robot: menuconfig (the site and env files) or monitor (live status).
#
#   scripts/console.sh menuconfig        # pconfig after `source scripts/env.sh`
#   scripts/console.sh monitor           # pmonitor; <r> writes a report file
#   scripts/console.sh monitor --once    # the report on stdout
#
# Same environment as pashupati.service (scripts/env.sh): RMW, domain, SITE_CONFIG, ROBOT_NS.
set -eo pipefail

mode="${1:-monitor}"
[[ $# -gt 0 ]] && shift
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$here/env.sh" > /dev/null
export ROBOT_NS="${ROBOT_NS:-robot}"
exec ros2 run dharma_console "$mode" \
    --product "$PASHUPATI_WS/src/pashupati/config/console.yaml" \
    --site "${SITE_CONFIG:-/etc/pashupati/site.yaml}" \
    --env "$PASHUPATI_ENV" "$@"
