import os

import pytest

from tb4_voicenav.command_parser import NOT_COMMAND, UNKNOWN_REGION, normalize_text, parse_command
from tb4_voicenav.region_store import RegionStore

REGIONS_JSON = os.path.join(os.path.dirname(__file__), '..', 'maps', 'regions.json')


@pytest.fixture
def region_store():
    return RegionStore().load(REGIONS_JSON)


def test_normalize_text_strips_diacritics():
    assert normalize_text('Phòng Khách') == 'phong khach'


def test_parse_di_den(region_store):
    result = parse_command('đi đến phòng khách', region_store)
    assert result.ok
    assert [r.name for r in result.targets] == ['Phòng khách']


def test_parse_di_toi(region_store):
    result = parse_command('đi tới phòng ngủ', region_store)
    assert result.ok
    assert [r.name for r in result.targets] == ['Phòng ngủ']


def test_parse_den_only(region_store):
    result = parse_command('đến phòng bếp', region_store)
    assert result.ok
    assert [r.name for r in result.targets] == ['Phòng bếp']


def test_parse_di_chuyen_den(region_store):
    result = parse_command('di chuyển đến phòng khách', region_store)
    assert result.ok
    assert [r.name for r in result.targets] == ['Phòng khách']


def test_parse_tu_den_queue(region_store):
    result = parse_command('đi từ phòng khách đến phòng bếp', region_store)
    assert result.ok
    assert [r.name for r in result.targets] == ['Phòng khách', 'Phòng bếp']


def test_parse_three_point_route(region_store):
    result = parse_command('đi từ phòng ngủ qua phòng khách rồi đến phòng bếp', region_store)
    assert result.ok
    assert [r.name for r in result.targets] == ['Phòng ngủ', 'Phòng khách', 'Phòng bếp']


def test_parse_qua_roi_den(region_store):
    result = parse_command('đi qua phòng khách rồi đến phòng bếp', region_store)
    assert result.ok
    assert [r.name for r in result.targets] == ['Phòng khách', 'Phòng bếp']


def test_parse_den_roi_chain(region_store):
    result = parse_command('đến phòng ngủ rồi phòng khách rồi phòng bếp', region_store)
    assert result.ok
    assert [r.name for r in result.targets] == ['Phòng ngủ', 'Phòng khách', 'Phòng bếp']


def test_parse_unknown_region(region_store):
    result = parse_command('đi đến hành lang', region_store)
    assert not result.ok
    assert result.error == UNKNOWN_REGION
    assert result.error_detail == 'hanh lang'


def test_parse_not_a_command(region_store):
    result = parse_command('hôm nay trời đẹp quá', region_store)
    assert not result.ok
    assert result.error == NOT_COMMAND


def test_parse_minor_typo_missing_letter(region_store):
    result = parse_command('đi đến phong khch', region_store)
    assert result.ok
    assert [r.name for r in result.targets] == ['Phòng khách']


def test_parse_minor_typo_extra_word(region_store):
    result = parse_command('đi đến phòng khách nhé', region_store)
    assert result.ok
    assert [r.name for r in result.targets] == ['Phòng khách']


def test_parse_alias_without_diacritics(region_store):
    result = parse_command('den phong ngu', region_store)
    assert result.ok
    assert [r.name for r in result.targets] == ['Phòng ngủ']


def test_parse_alias_english(region_store):
    result = parse_command('đi đến living room', region_store)
    assert result.ok
    assert [r.name for r in result.targets] == ['Phòng khách']


def test_parse_misheard_hong(region_store):
    result = parse_command('đi đến phòng hộng', region_store)
    assert result.ok
    assert [r.name for r in result.targets] == ['Phòng khách']


def test_parse_misheard_mu(region_store):
    result = parse_command('đi đến phòng mũ', region_store)
    assert result.ok
    assert [r.name for r in result.targets] == ['Phòng ngủ']


def test_parse_misheard_pre(region_store):
    result = parse_command('đi đến phòng pré', region_store)
    assert result.ok
    assert [r.name for r in result.targets] == ['Phòng bếp']


def test_parse_misheard_den_bep(region_store):
    result = parse_command('đi đen phòng bếp', region_store)
    assert result.ok
    assert [r.name for r in result.targets] == ['Phòng bếp']
