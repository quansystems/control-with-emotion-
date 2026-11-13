"""Occupancy-map geometry, geodesic segmentation and validated semantic labels."""

import hashlib
import heapq
import math
from pathlib import Path
import re
import unicodedata

import cv2
import numpy as np
import yaml


SPOKEN_LETTERS = {
    'A': ('a',), 'B': ('bê', 'bờ'), 'C': ('xê', 'cờ'), 'D': ('đê', 'dê', 'đờ', 'dờ'),
    'E': ('e',), 'F': ('ép',), 'G': ('giê',), 'H': ('hát',),
    'I': ('i',), 'J': ('giây',), 'K': ('ca',), 'L': ('e lờ',),
    'M': ('em mờ',), 'N': ('en nờ',), 'O': ('o',), 'P': ('pê',),
    'Q': ('quy',), 'R': ('e rờ',), 'S': ('ét',), 'T': ('tê',),
    'U': ('u',), 'V': ('vê',), 'W': ('vê kép',), 'X': ('ích',),
    'Y': ('y',), 'Z': ('dét',),
}

ENGLISH_LETTERS = {
    'A': ('ay',), 'B': ('bee', 'be'), 'C': ('see', 'sea'), 'D': ('dee',),
    'E': ('ee',), 'F': ('ef',), 'G': ('gee',), 'H': ('aitch',),
    'I': ('eye',), 'J': ('jay',), 'K': ('kay',), 'L': ('el',),
    'M': ('em',), 'N': ('en',), 'O': ('oh',), 'P': ('pee',),
    'Q': ('cue',), 'R': ('are',), 'S': ('ess',), 'T': ('tee',),
    'U': ('you',), 'V': ('vee',), 'W': ('double u',), 'X': ('ex',),
    'Y': ('why',), 'Z': ('zee', 'zed'),
}


def default_regions_file():
    """Use one installed catalog by default across speech, navigation and demos."""
    workspace_file = Path(__file__).resolve().parents[2] / 'Map Partitioning/generated/regions.yaml'
    if workspace_file.is_file():
        return str(workspace_file)
    from ament_index_python.packages import get_package_share_directory
    return str(Path(get_package_share_directory('robot_node')) / 'semantic_map' / 'regions.yaml')


def normalize(text):
    text = unicodedata.normalize('NFD', str(text).lower().replace('đ', 'd'))
    text = ''.join(c for c in text if unicodedata.category(c) != 'Mn')
    return ' '.join(re.sub(r'[^a-z0-9\s]', ' ', text).split())


class OccupancyMap:
    def __init__(self, filename):
        self.filename = Path(filename).resolve()
        self.meta = yaml.safe_load(self.filename.read_text())
        if self.meta.get('mode', 'trinary') != 'trinary':
            raise ValueError('Only trinary occupancy maps are supported.')
        image_path = self.filename.parent / self.meta['image']
        self.image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if self.image is None:
            raise ValueError(f'Cannot read map image: {image_path}')
        self.resolution = float(self.meta['resolution'])
        self.origin = tuple(float(v) for v in self.meta['origin'])
        if (len(self.origin) != 3 or self.resolution <= 0 or
                not all(math.isfinite(v) for v in (*self.origin, self.resolution))):
            raise ValueError('Invalid map resolution/origin.')
        occupied = float(self.meta['occupied_thresh'])
        free = float(self.meta['free_thresh'])
        if not 0 <= free < occupied <= 1:
            raise ValueError('Invalid occupancy thresholds.')
        probability = self.image.astype(float) / 255.0
        if not self.meta.get('negate', 0):
            probability = 1.0 - probability
        self.free = probability < free
        # Pad unknown/outside as obstacles so clearance also respects image edges.
        padded = np.pad(self.free.astype(np.uint8), 1)
        self.clearance = cv2.distanceTransform(padded, cv2.DIST_L2, 5)[1:-1, 1:-1]
        self.clearance *= self.resolution
        self.height, self.width = self.free.shape
        signature = {k: self.meta.get(k, 'trinary' if k == 'mode' else None)
                     for k in ('resolution', 'origin', 'negate', 'free_thresh',
                               'occupied_thresh', 'mode')}
        self.fingerprint = hashlib.sha256(
            yaml.safe_dump(signature, sort_keys=True).encode() + self.image.tobytes()
        ).hexdigest()

    def pixel_to_world(self, u, v):
        x = (float(u) + 0.5) * self.resolution
        y = (self.height - float(v) - 0.5) * self.resolution
        ox, oy, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        return [ox + c*x - s*y, oy + s*x + c*y]

    def world_to_pixel(self, x, y):
        ox, oy, yaw = self.origin
        dx, dy = x - ox, y - oy
        c, s = math.cos(yaw), math.sin(yaw)
        return (math.floor((c*dx + s*dy) / self.resolution),
                self.height - 1 - math.floor((-s*dx + c*dy) / self.resolution))

    def valid_goal(self, x, y, clearance):
        u, v = self.world_to_pixel(x, y)
        return (0 <= u < self.width and 0 <= v < self.height and
                self.free[v, u] and self.clearance[v, u] >= clearance)


def partition(grid, seeds=None, opening_radius=0.4, min_area=0.5):
    """Split known free cells by shortest 4-connected distance from region seeds.

    Automatic seeds are components remaining after morphological opening.
    Unseeded disconnected components stay unassigned (0), never bridged by walls.
    """
    labels = np.zeros(grid.free.shape, dtype=np.int32)
    queue = []
    if seeds is None:
        if opening_radius <= 0 or min_area <= 0:
            raise ValueError('opening_radius and min_area must be positive.')
        radius = max(1, round(opening_radius / grid.resolution))
        core = cv2.morphologyEx(grid.free.astype(np.uint8), cv2.MORPH_OPEN,
                               cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                        (2*radius+1, 2*radius+1)))
        count, components, stats, _ = cv2.connectedComponentsWithStats(core, 4)
        seeds = []
        for component in range(1, count):
            if stats[component, cv2.CC_STAT_AREA] * grid.resolution**2 < min_area:
                continue
            score = np.where(components == component, grid.clearance, -1)
            v, u = np.unravel_index(score.argmax(), score.shape)
            index = len(seeds)
            region_id = chr(65 + index) if index < 26 else f'R{index + 1}'
            seeds.append({'id': region_id, 'name': f'Khu vực {region_id}',
                          'seed_pixel': [int(u), int(v)], 'aliases': []})
    if not seeds:
        raise ValueError('No region seeds. Reduce opening radius/min area or supply seeds.')
    for index, seed in enumerate(seeds, 1):
        u, v = seed.get('seed_pixel', (None, None))
        if u is None:
            u, v = grid.world_to_pixel(*seed['seed_world'])
        u, v = int(u), int(v)
        if not (0 <= u < grid.width and 0 <= v < grid.height and grid.free[v, u]):
            raise ValueError(f'Seed {seed["id"]} is not in known free space.')
        if labels[v, u]:
            raise ValueError('Region seeds must be distinct.')
        labels[v, u] = index
        heapq.heappush(queue, (0, index, v, u))
    while queue:
        distance, index, v, u = heapq.heappop(queue)
        for nv, nu in ((v-1, u), (v+1, u), (v, u-1), (v, u+1)):
            if (0 <= nv < grid.height and 0 <= nu < grid.width and
                    grid.free[nv, nu] and labels[nv, nu] == 0):
                labels[nv, nu] = index
                heapq.heappush(queue, (distance + 1, index, nv, nu))
    return labels, seeds


def select_region_goal(grid, mask, clearance, policy='center', polygon=None):
    """Nearest safe cell to the region centroid, with an explicit legacy policy."""
    candidates = mask & grid.free & (grid.clearance >= clearance)
    if not candidates.any():
        raise ValueError(f'Vùng không có đích an toàn với clearance {clearance:g} m.')
    if policy == 'max_clearance':
        score = np.where(candidates, grid.clearance, -1)
        v, u = np.unravel_index(score.argmax(), score.shape)
    elif policy == 'center':
        rows, columns = np.nonzero(mask)
        center_u, center_v = float(columns.mean()), float(rows.mean())
        if polygon is not None:
            moments = cv2.moments(np.asarray(polygon, dtype=np.float32))
            if abs(moments['m00']) > 1e-9:
                center_u = moments['m10']/moments['m00']
                center_v = moments['m01']/moments['m00']
        rows, columns = np.nonzero(candidates)
        distances = (columns-center_u)**2 + (rows-center_v)**2
        # Physical clearance breaks ties; distance to the center stays primary.
        index = np.lexsort((-grid.clearance[rows, columns], distances))[0]
        u, v = columns[index], rows[index]
    else:
        raise ValueError(f'Unknown goal policy: {policy}')
    return int(u), int(v)


def region_records(grid, labels, seeds, clearance):
    if not math.isfinite(clearance) or clearance <= 0:
        raise ValueError('Goal clearance must be positive and finite.')
    regions = []
    for index, seed in enumerate(seeds, 1):
        mask = labels == index
        policy = seed.get('goal_policy', 'center')
        try:
            u, v = select_region_goal(grid, mask, clearance, policy, seed.get('polygon_pixels'))
        except ValueError as exc:
            raise ValueError(f'Region {seed["id"]} has no goal or invalid policy: {exc}') from exc
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_LIST,
                                      cv2.CHAIN_APPROX_SIMPLE)
        outlines = [[grid.pixel_to_world(int(p[0][0]), int(p[0][1])) for p in contour]
                    for contour in contours if len(contour) >= 3]
        x, y = grid.pixel_to_world(u, v)
        regions.append({'id': str(seed['id']), 'name': str(seed['name']),
                        'aliases': seed.get('aliases', []),
                        'goal': {'x': x, 'y': y, 'yaw': 0.0},
                        'goal_policy': policy,
                        'area_m2': float(mask.sum() * grid.resolution**2),
                        'outlines': outlines})
    return regions


class RegionCatalog:
    def __init__(self, filename, map_override=None):
        filename = Path(filename).resolve()
        self.filename = filename
        self.revision = hashlib.sha256(filename.read_bytes()).hexdigest()
        self.data = yaml.safe_load(filename.read_text())
        self.grid = OccupancyMap(map_override or filename.parent / self.data['map_yaml'])
        if self.data['map_fingerprint'] != self.grid.fingerprint:
            raise ValueError('Map changed; regenerate regions for this map.')
        self.clearance = float(self.data['goal_clearance_m'])
        if not math.isfinite(self.clearance) or self.clearance <= 0:
            raise ValueError('Invalid goal clearance.')
        self.regions = self.data['regions']
        if not isinstance(self.regions, list) or not self.regions:
            raise ValueError('No semantic regions.')
        self.by_id, self.aliases = {}, {}
        for region in self.regions:
            region_id = region['id']
            if not region_id or region_id in self.by_id or not region['name'].strip():
                raise ValueError('Region IDs/names must be nonempty; IDs must be unique.')
            goal = region['goal']
            if not all(math.isfinite(float(goal[k])) for k in ('x', 'y', 'yaw')):
                raise ValueError(f'Non-finite goal: {region_id}')
            if not self.grid.valid_goal(goal['x'], goal['y'], self.clearance):
                raise ValueError(f'Unsafe goal: {region_id}')
            self.by_id[region_id] = region
            aliases = [region['name'], *region.get('aliases', [])]
            for spoken_id in (region_id, *SPOKEN_LETTERS.get(region_id.upper(), ())):
                aliases.extend(f'{prefix}{spoken_id}' for prefix in
                               ('', 'khu vực ', 'vùng ', 'khu ', 'phòng ', 'hành lang '))
            for spoken_id in (region_id, *ENGLISH_LETTERS.get(region_id.upper(), ())):
                aliases.extend(f'{prefix}{spoken_id}' for prefix in
                               ('', 'area ', 'region ', 'zone ', 'room '))
            for alias in aliases:
                key = normalize(alias)
                if not key or (key in self.aliases and self.aliases[key] != region_id):
                    raise ValueError(f'Ambiguous or empty region alias: {alias}')
                self.aliases[key] = region_id

    def speech_prompt(self, language='en'):
        """Vocabulary only: do not provide an expected transcript to Whisper."""
        if language == 'en':
            names = [f'Area {r["id"]}' + (f' ({r["name"]})' if r['name'] != r['id'] else '')
                     for r in self.regions]
            return ('Robot navigation vocabulary: go to, move to, navigate to, go from, '
                    'via, through, then to, stop, cancel navigation. Areas: ' + ', '.join(names) + '.')
        if language != 'vi':
            raise ValueError('Supported speech languages: en, vi.')
        descriptions = []
        for region in self.regions:
            region_id = region['id']
            spoken = SPOKEN_LETTERS.get(region_id.upper(), (region_id,))[0]
            descriptions.extend([f'khu vực {region_id}', f'khu vực {spoken}'])
            if region['name'] != region_id:
                descriptions.append(region['name'])
        return ('Từ vựng điều hướng: đi đến, di chuyển từ, qua, rồi đến. '
                'Tên các khu vực: ' + ', '.join(dict.fromkeys(descriptions)) + '.')

    def demo_scenarios(self, language='en'):
        """Generate demonstrations from configured IDs, without assuming room meanings."""
        if language == 'en':
            ordered = sorted(self.by_id)
            scenarios = [{'key': f'{i:02d}_region_{r}', 'title': f'Go to area {r}',
                          'command': f'Go to area {r}.', 'route': [r], 'invalid': False}
                         for i, r in enumerate(ordered, 1)]
            if len(ordered) >= 2:
                route = ordered[:4]
                command = f'Go from area {route[0]}'
                for region_id in route[1:-1]:
                    command += f' via area {region_id}'
                command += f' then to area {route[-1]}.'
                scenarios.append({'key': 'multi_stop', 'title': ' → '.join(route),
                                  'command': command, 'route': route, 'invalid': False})
            unknown = 'unknown area'
            while normalize(unknown) in self.aliases:
                unknown += ' other'
            scenarios.append({'key': 'unknown_region', 'title': 'Unknown area',
                              'command': f'Go to {unknown}.', 'route': [], 'invalid': True})
            for scenario in scenarios:
                scenario['spoken_command'] = scenario['command'].replace(' via ', ', via ').replace(
                    ' then to ', ', then to ')
            return scenarios
        if language != 'vi':
            raise ValueError('Supported speech languages: en, vi.')
        ordered = sorted(self.by_id)
        scenarios = []
        for index, region_id in enumerate(ordered, 1):
            scenarios.append({'key': f'{index:02d}_region_{region_id}',
                              'title': f'Đến khu vực {region_id}',
                              'command': f'Đi đến khu vực {region_id}.',
                              'route': [region_id], 'invalid': False})
        if len(ordered) >= 2:
            route = ordered[:4]
            command = f'Đi từ khu vực {route[0]}'
            for region_id in route[1:-1]:
                command += f' qua khu vực {region_id}'
            command += f' rồi đến khu vực {route[-1]}.'
            scenarios.append({'key': 'multi_stop', 'title': ' → '.join(route),
                              'command': command, 'route': route, 'invalid': False})
        unknown = 'khu vực không tồn tại'
        while normalize(unknown) in self.aliases:
            unknown += ' khác'
        scenarios.append({'key': 'unknown_region', 'title': 'Khu vực không tồn tại',
                          'command': f'Đi đến {unknown}.', 'route': [], 'invalid': True})
        pattern = r'khu vực (' + '|'.join(re.escape(r) for r in sorted(ordered, key=len, reverse=True)) + r')(?=[ .]|$)'
        for scenario in scenarios:
            scenario['spoken_command'] = re.sub(pattern, lambda match: 'khu vực ' +
                SPOKEN_LETTERS.get(match[1].upper(), (match[1],))[0], scenario['command'])
            if len(scenario['route']) > 1:
                scenario['spoken_command'] = scenario['spoken_command'].replace(
                    'Đi từ ', 'Di chuyển từ ', 1).replace(' qua ', ', qua ').replace(
                    ' rồi đến ', ', rồi đến ')
        return scenarios

    def parse(self, command):
        """Consume the complete grammar; never silently omit an unknown waypoint."""
        text = normalize(command)
        if text.startswith('please '):
            text = text[7:]
        if text.endswith(' please'):
            text = text[:-7]
        if text in ('dung', 'dung lai', 'huy', 'huy dieu huong', 'stop', 'stop moving',
                    'stop navigation', 'cancel', 'cancel navigation'):
            return []
        english = re.fullmatch(r'(?:go|move|navigate|travel) (to|from) (.+)', text)
        take_me = re.fullmatch(r'take me to (.+)', text)
        if english or take_me:
            origin = english is not None and english[1] == 'from'
            body = english[2] if english else take_me[1]
            # Natural English may put the via stop after the final destination.
            # "from A to B via D" must visit A,D,B, not A,B,D.
            chunks = [body]
            if not re.search(r'\bthen\b', body):
                suffix = re.fullmatch(r'(.+?)\s+(?:via|through)\s+(.+)', body)
                if suffix:
                    destination_part, via_part = suffix.groups()
                    start_dest = re.fullmatch(r'(.+?)\s+to\s+(.+)', destination_part) if origin else None
                    if start_dest:
                        chunks = [start_dest[1], via_part, start_dest[2]]
                    elif not origin:
                        chunks = [via_part, destination_part]
            parts = []
            separator = r'\s+(?:and then go to|and then to|then go to|then to|and then|via|through|then|to|and)\s+'
            for chunk in chunks:
                parts.extend([chunk] if chunk in self.aliases else re.split(separator, chunk))
            route = []
            for part in parts:
                if part not in self.aliases:
                    raise ValueError(f'Unknown area: {part}')
                route.append(self.aliases[part])
            if origin and len(route) < 2:
                raise ValueError('A command starting with from needs at least two areas.')
            return route
        match = re.fullmatch(r'(?:di|di chuyen)(?: tu | den )(.+)', text)
        if not match:
            raise ValueError('Câu lệnh không hợp lệ. Ví dụ: Đi đến khu vực A.')
        parts = re.split(r'\s+(?:roi den|roi di den|roi|qua|den|toi)\s+', match[1])
        route = []
        for part in parts:
            if part not in self.aliases:
                raise ValueError(f'Khu vực không tồn tại: {part}')
            route.append(self.aliases[part])
        if ' tu ' in f' {text} ' and len(route) < 2:
            raise ValueError('Lệnh đi từ cần ít nhất hai khu vực.')
        return route
