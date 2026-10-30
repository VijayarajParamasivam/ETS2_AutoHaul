"""The launcher's city and company boxes: what they suggest, and what they hand to drive.py."""

from __future__ import annotations

import json
import sys
import tkinter as tk
from pathlib import Path
from types import SimpleNamespace

import pytest

from autohaul.world.place_search import Option, PlaceIndex, exact, fold, search

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))


@pytest.fixture
def index(tmp_path):
    raw = tmp_path / "raw"
    raw.mkdir()
    (raw / "europe-cities.json").write_text(json.dumps([
        {"token": "nurnberg", "name": "N\u00fcrnberg", "countryToken": "germany"},
        {"token": "lodz", "name": "\u0141\u00f3d\u017a", "countryToken": "poland"},
        {"token": "new_castle", "name": "Newcastle-upon-Tyne", "countryToken": "uk"},
        {"token": "berlin", "name": "Berlin", "countryToken": "germany"},
        {"token": "bern", "name": "Bern", "countryToken": "switzerland"},
        {"token": "saint_peter", "name": "Saint Petersburg", "countryToken": "russia"},
        {"token": "empty", "name": "No Companies", "countryToken": "germany"}]), encoding="utf-8")
    (raw / "europe-companies.json").write_text(json.dumps([
        {"token": "lkwlog", "cityToken": "nurnberg"}, {"token": "lkwlog", "cityToken": "nurnberg"},   # two depots: one choice
        {"token": "kathode", "cityToken": "nurnberg"}, {"token": "nbfc", "cityToken": "berlin"},
        {"token": "lkwlog", "cityToken": "berlin"}]), encoding="utf-8")
    (raw / "europe-companyDefs.json").write_text(json.dumps([
        {"token": "lkwlog", "name": "LkwLog GmbH"}, {"token": "kathode", "name": "Kathode"}, {"token": "nbfc", "name": "NBFC"}]),
        encoding="utf-8")
    return PlaceIndex(tmp_path)


def labels(found):
    return [o.label for o in found]


def test_accents_and_case_dont_matter():
    assert fold("N\u00dcRNBERG") == "nurnberg" and fold("\u0141\u00f3d\u017a") == "lodz" and fold(" Stra\u00dfe ") == "strasse"


def test_cities_are_suggested_while_typing(index):
    assert labels(search("nurn", index.cities)) == ["N\u00fcrnberg, Germany"]
    assert labels(search("LODZ", index.cities)) == ["\u0141\u00f3d\u017a, Poland"]
    assert labels(search("ber", index.cities)) == ["Berlin, Germany", "Bern, Switzerland", "Nürnberg, Germany"]
    assert labels(search("zzz", index.cities)) == []
    assert labels(search("newcastle", index.cities)) == ["Newcastle-upon-Tyne, UK"]


def test_the_best_match_comes_first(index):
    assert search("bern", index.cities)[0].token == "bern"                 # the name itself, before Berlin's "ber"
    assert search("peters", index.cities)[0].token == "saint_peter"        # a later word's start
    assert search("upon", index.cities)[0].token == "new_castle"           # (after a hyphen too)
    assert search("german", index.cities)                                  # the country is in the label
    options = [Option("a", "Xberlin", "Xberlin"), Option("b", "Berlin Ost", "Berlin Ost")]
    assert search("berlin", options)[0].token == "b"                       # how it starts, before somewhere inside


def test_nothing_typed_lists_everything_up_to_the_limit(index):
    assert len(search("", index.cities, limit=3)) == 3
    assert labels(search("", index.companies("nurnberg"))) == ["Kathode", "LkwLog GmbH"]


def test_companies_are_those_of_the_city(index):
    assert labels(search("", index.companies("berlin"))) == ["LkwLog GmbH", "NBFC"]
    assert index.companies("empty") == [] and index.companies("nowhere") == []
    assert len(index.companies("nurnberg")) == 2                           # two depots of one company: one choice


def test_a_box_recognises_a_label_a_name_or_a_token(index):
    for text in ("N\u00fcrnberg, Germany", "nurnberg", "N\u00dcRNBERG"):
        assert exact(text, index.cities).token == "nurnberg"
    assert exact("", index.cities) is None and exact("nur", index.cities) is None


def test_the_real_map_has_companies_for_its_cities():
    from autohaul.infra.config import load_config
    from autohaul.infra.paths import map_dir
    try:
        real = PlaceIndex(map_dir(load_config()))
    except OSError:
        pytest.skip("no parsed map here")
    assert len(real.cities) > 100
    assert any(real.companies(c.token) for c in real.cities)
    assert search("nurn", real.cities)[0].token == "nurnberg"
    assert all(o.name for c in real.cities for o in real.companies(c.token))
    from autohaul.world.places import Places
    places = Places(map_dir(load_config()))                   # whatever the boxes offer, drive.py can find
    for c in real.cities:
        assert places.resolve(c.token) is not None, c.token
        for o in real.companies(c.token):
            assert places.resolve(f"{c.token}:{o.token}") is not None, (c.token, o.token)


def test_what_goes_to_drive_py():
    import launcher
    city, company = Option("berlin", "Berlin", "Berlin, Germany"), Option("nbfc", "NBFC", "NBFC")
    assert launcher.place_spec(city, company) == "berlin:nbfc"
    assert launcher.place_spec(city, None) == "berlin"
    assert launcher.place_spec(None, company) == ""


@pytest.fixture
def root():
    try:
        r = tk.Tk()
    except tk.TclError:
        pytest.skip("no display")
    r.withdraw()
    yield r
    r.destroy()


def test_the_box_suggests_takes_a_pick_and_closes(root, index):
    import launcher
    box = launcher.SuggestEntry(root, lambda: index.cities)
    box.pack()
    box.text.set("ber")
    box.refresh()
    assert box.popup is not None and box.listbox.size() == 3
    assert box.listbox.curselection() == (0,)                              # the best one is ready for Enter
    box._move(1)
    assert box.listbox.curselection() == (1,)
    box._accept()
    assert box.popup is None and box.text.get() == "Bern, Switzerland" and box.choice().token == "bern"
    box.text.set("zzz")
    box.refresh()
    assert box.popup is None                                               # nothing to suggest: no list
    box.text.set("Bern, Switzerland")
    box.refresh()
    assert box.popup is None                                               # already chosen: nothing more to say


def test_the_launcher_boxes_work_together(root, index, monkeypatch):
    import launcher
    monkeypatch.setattr(launcher, "PlaceIndex", lambda _: index)
    monkeypatch.setattr(launcher, "load_config", lambda: None)
    monkeypatch.setattr(launcher, "map_dir", lambda _: None)
    monkeypatch.setattr(launcher.Launcher, "_load", staticmethod(lambda: {"mode": "to", "place": "berlin:nbfc"}))
    monkeypatch.setattr(root, "after", lambda *a, **k: None)
    ui = launcher.Launcher(root)
    assert ui.city.text.get() == "Berlin, Germany" and ui.company.text.get() == "NBFC"     # the last choice
    assert ui._spec() == "berlin:nbfc" and "NBFC in Berlin" in ui.destination.get()
    ui.city.text.set("N\u00fcrnberg, Germany")
    ui.city.picked = None
    ui._city_changed()
    assert ui.company.text.get() == ""                                     # NBFC isn't in N\u00fcrnberg
    assert ui._spec() == "nurnberg" and "centre" in ui.destination.get()
    ui.company.text.set("kath")
    ui._show_destination()
    assert ui.destination.get() == ""                         # still typing: nothing's wrong yet
    ui.company.refresh()
    ui.company._accept()
    assert ui._spec() == "nurnberg:kathode"
    ui.company.text.set("zzz")
    ui._show_destination()
    assert "no company like that" in ui.destination.get()
    ui.city.text.set("nowhere")
    ui._city_changed()
    assert str(ui.company.entry.cget("state")) == "disabled" and ui._spec() == ""
