"""Current operator catalog and generated speech/demo configuration stay aligned."""

from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest
import yaml

from robot_node.semantic_map import RegionCatalog, SPOKEN_LETTERS


def test_current_saved_regions_have_matching_goals_and_commands():
    path = Path(__file__).resolve().parents[2]/'Map Partitioning/generated/regions.yaml'
    catalog = RegionCatalog(path)
    labels = np.load(path.parent/'labels.npy', allow_pickle=False)
    assert labels.shape == catalog.grid.free.shape
    assert not np.any((labels > 0) & ~catalog.grid.free)
    assert labels.max() == len(catalog.regions)
    for index, region in enumerate(catalog.regions, 1):
        u, v = catalog.grid.world_to_pixel(region['goal']['x'], region['goal']['y'])
        assert labels[v, u] == index
        assert catalog.parse(f'Đi đến khu vực {region["id"]}') == [region['id']]
        for spoken in SPOKEN_LETTERS.get(region['id'].upper(), ()):
            assert catalog.parse(f'Đi đến vùng {spoken}') == [region['id']]
    for scenario in catalog.demo_scenarios():
        if scenario['invalid']:
            with pytest.raises(ValueError):
                catalog.parse(scenario['command'])
        else:
            assert catalog.parse(scenario['command']) == scenario['route']
            assert catalog.parse(scenario['spoken_command']) == scenario['route']


def test_prompt_and_scenarios_follow_renamed_or_added_regions(sample_catalog_file):
    data = yaml.safe_load(sample_catalog_file.read_text())
    data['regions'][0].update(id='Room_7', name='Xưởng chế tạo', aliases=['xưởng'])
    fourth = deepcopy(data['regions'][1])
    fourth.update(id='D', name='Khu mới', aliases=[])
    data['regions'].append(fourth)
    sample_catalog_file.write_text(yaml.safe_dump(data, allow_unicode=True))
    catalog = RegionCatalog(sample_catalog_file)
    assert 'Xưởng chế tạo' in catalog.speech_prompt()
    assert 'Phòng học' not in catalog.speech_prompt()
    assert catalog.parse('Đi đến khu vực đê') == ['D']
    scenarios = catalog.demo_scenarios()
    assert [s['route'][0] for s in scenarios[:-2]] == sorted(catalog.by_id)
    assert len(scenarios) == len(catalog.regions)+2
    for scenario in scenarios[:-1]:
        assert catalog.parse(scenario['command']) == scenario['route']
        assert catalog.parse(scenario['spoken_command']) == scenario['route']
