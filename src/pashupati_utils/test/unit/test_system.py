from __future__ import annotations

from pashupati_utils.core.system import FIGURES, Finding, system_figures, topic_rates


def report():
    return [
        Finding.of('health: system:temperature', {'value': '61.3', 'temp_cpu': '61.3'}),
        Finding.of(
            'health: system:memory', {'value': '41.25', 'used_gb': '6.4', 'total_gb': '15.3'}
        ),
        Finding.of('health: system:cpu', {'value': '23.4'}),
        Finding.of('health: system:load', {'value': '0.5'}),
        Finding.of('health: sensor:/livox/lidar', {'hz': '9.97', 'min_hz': '5.0'}),
        Finding.of('health: sensor:/Odometry', {'min_hz': '8.0'}),
        Finding.of('health: node:/robot/x', {}),
    ]


def test_finding_names_lose_the_prefix():
    assert Finding.of('health: system:cpu', {}).name == 'system:cpu'
    assert Finding.of('plain', {}).name == 'plain'


def test_system_figures_of_a_report():
    figures = system_figures(report(), cpus=4)
    assert figures['cpu_temp_c'] == 61.3
    assert figures['memory_percent'] == 41.2
    assert (figures['memory_used_gb'], figures['memory_total_gb']) == (6.4, 15.3)
    assert figures['cpu_percent'] == 23.4
    assert figures['gpu_percent'] is None  # no Jetson GPU reported
    assert figures['load'] == 2.0  # per CPU back to the load average
    assert set(figures) == set(FIGURES)


def test_missing_figures_are_none():
    assert all(v is None for v in system_figures([]).values())


def test_topic_rates():
    assert topic_rates(report()) == {'/livox/lidar': 10.0, '/Odometry': 0.0}
