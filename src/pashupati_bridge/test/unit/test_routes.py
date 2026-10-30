from __future__ import annotations

import os
import time

from pashupati_bridge.core.routes import list_routes, route_path
import pytest


def test_routes_are_listed_relative_and_newest_first(tmp_path):
    (tmp_path / 'lobby' / 'routes').mkdir(parents=True)
    old, new = tmp_path / 'lobby' / 'routes' / 'a.csv', tmp_path / 'lobby' / 'routes' / 'b.csv'
    old.write_text('x')
    new.write_text('x')
    os.utime(old, (time.time() - 100, time.time() - 100))
    (tmp_path / 'lobby' / 'tags.json').write_text('{}')
    assert list_routes(str(tmp_path)) == ['lobby/routes/b.csv', 'lobby/routes/a.csv']
    assert list_routes(str(tmp_path / 'missing')) == []


def test_only_csv_files_inside_the_sites(tmp_path):
    assert route_path(str(tmp_path), 'lobby/routes/a.csv') == 'lobby/routes/a.csv'
    assert route_path(str(tmp_path), str(tmp_path / 'lobby' / 'a.csv')) == 'lobby/a.csv'
    for bad in ('../x.csv', '/etc/passwd', 'lobby/tags.json', '', None):
        with pytest.raises(ValueError):
            route_path(str(tmp_path), bad)
