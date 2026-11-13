"""English commands must consume every area and preserve intended visit order."""

from copy import deepcopy

import pytest
import yaml

from robot_node.semantic_map import RegionCatalog


@pytest.fixture
def english_catalog(sample_catalog_file):
    data = yaml.safe_load(sample_catalog_file.read_text())
    region = deepcopy(data['regions'][0])
    region.update(id='D', name='D', aliases=[])
    data['regions'].append(region)
    sample_catalog_file.write_text(yaml.safe_dump(data, allow_unicode=True))
    return RegionCatalog(sample_catalog_file)


@pytest.mark.parametrize('text,route', [
    ('Go to area A.', ['A']), ('Move to region B', ['B']),
    ('Navigate to zone C', ['C']), ('Please take me to room D.', ['D']),
    ('Go to area bee', ['B']), ('Go to area see', ['C']), ('Go to area dee', ['D']),
    ('Go from area A via area D then to area B.', ['A', 'D', 'B']),
    ('Move from area A to area B via area D', ['A', 'D', 'B']),
    ('Go to area B through area D', ['D', 'B']),
    ('Go from A to B via D and C', ['A', 'D', 'C', 'B']),
    ('Go to area A then to area B then go to area C', ['A', 'B', 'C']),
    ('Go to A and then B', ['A', 'B']), ('Navigate from A through D to B', ['A', 'D', 'B']),
    ('Stop.', []), ('Cancel navigation', []), ('Please stop moving', []),
])
def test_english_commands(english_catalog, text, route):
    assert english_catalog.parse(text) == route


@pytest.mark.parametrize('text', [
    'Go to area Z', 'Go from A via Z to B', 'Go from A to B via Z',
    'Go from A', 'Do not go to A', 'Go to A and Z', 'Go to A then',
    'area A', 'Go to A without going to B', 'Go to area unknown',
])
def test_bad_english_command_is_rejected_in_full(english_catalog, text):
    with pytest.raises(ValueError):
        english_catalog.parse(text)


def test_english_scenarios_and_context_use_configured_areas(english_catalog):
    assert 'Robot navigation' in english_catalog.speech_prompt('en')
    for scenario in english_catalog.demo_scenarios('en'):
        if scenario['invalid']:
            with pytest.raises(ValueError):
                english_catalog.parse(scenario['spoken_command'])
        else:
            assert english_catalog.parse(scenario['command']) == scenario['route']
    # Existing Vietnamese commands are still an explicitly selectable ASR option.
    assert english_catalog.parse('Đi đến khu vực bê') == ['B']
    assert 'Tên các khu vực' in english_catalog.speech_prompt('vi')
