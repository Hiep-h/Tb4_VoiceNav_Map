import difflib
import json

from tb4_voicenav.command_parser import normalize_text

DEFAULT_FUZZY_THRESHOLD = 0.72


class Region:
    def __init__(self, region_id, name, aliases, center_px, goal_px, goal_m, area_px):
        self.region_id = region_id
        self.name = name
        self.aliases = aliases
        self.center_px = center_px
        self.goal_px = goal_px
        self.goal_m = goal_m
        self.area_px = area_px


class RegionStore:
    def __init__(self, fuzzy_threshold=DEFAULT_FUZZY_THRESHOLD):
        self.regions = {}
        self.fuzzy_threshold = fuzzy_threshold

    def load(self, regions_json_path):
        with open(regions_json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        for entry in data.get('regions', []):
            region = Region(
                entry['id'],
                entry['name'],
                entry.get('aliases', []),
                entry['center_px'],
                entry['goal_px'],
                entry['goal_m'],
                entry.get('area_px', 0),
            )
            self.regions[region.region_id] = region
        return self

    def find_by_alias(self, phrase):
        """So khop mo CUM tu phrase voi ten/bi danh moi vung. Tra ve (Region, score)
        hoac (None, best_score) neu khong vung nao vuot nguong fuzzy_threshold."""
        if not phrase:
            return None, 0.0
        norm_phrase = normalize_text(phrase)
        last_phrase = norm_phrase.split()[-1] if norm_phrase.split() else norm_phrase
        best_region, best_score = None, 0.0
        for region in self.regions.values():
            for candidate in [region.name] + list(region.aliases):
                norm_candidate = normalize_text(candidate)
                score = difflib.SequenceMatcher(None, norm_phrase, norm_candidate).ratio()
                if norm_candidate in norm_phrase or norm_phrase in norm_candidate:
                    score = max(score, 0.9)
                last_candidate = norm_candidate.split()[-1] if norm_candidate.split() \
                    else norm_candidate
                last_ratio = difflib.SequenceMatcher(None, last_phrase, last_candidate).ratio()
                if last_ratio >= 0.8:
                    score = max(score, 0.85 + 0.1 * last_ratio)
                if score > best_score:
                    best_score, best_region = score, region
        if best_score >= self.fuzzy_threshold:
            return best_region, best_score
        return None, best_score

    def get(self, region_id):
        return self.regions.get(region_id)
