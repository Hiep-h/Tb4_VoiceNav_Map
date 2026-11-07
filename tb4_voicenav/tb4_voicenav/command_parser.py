import re
import unicodedata


def strip_diacritics(text):
    text = text.replace('đ', 'd').replace('Đ', 'D')
    normalized = unicodedata.normalize('NFD', text)
    without_marks = ''.join(c for c in normalized if unicodedata.category(c) != 'Mn')
    return without_marks


def normalize_text(text):
    return strip_diacritics(text).lower().strip()


NOT_COMMAND = 'not_a_command'
UNKNOWN_REGION = 'unknown_region'

COMMAND_PREFIXES = ('tu ', 'den ', 'toi ', 'di ')
# cat tach cac diem den: "di tu X qua Y roi den Z", "di qua Y roi den Z",
# "den X roi Y roi Z", so diem tuy y.
CONNECTOR_RE = re.compile(r'\bdi\s+chuyen\b|\btu\b|\bden\b|\btoi\b|\bqua\b|\broi\b|\bdi\b')


class ParseResult:
    def __init__(self, raw_text, targets=None, error=None, error_detail=None):
        self.raw_text = raw_text
        self.targets = targets or []
        self.error = error
        self.error_detail = error_detail

    @property
    def ok(self):
        return self.error is None


def parse_command(text, region_store):
    normalized = normalize_text(text)
    if not normalized.startswith(COMMAND_PREFIXES):
        return ParseResult(text, error=NOT_COMMAND)

    phrases = [p.strip() for p in CONNECTOR_RE.split(normalized) if p.strip()]
    if not phrases:
        return ParseResult(text, error=NOT_COMMAND)

    targets = []
    for phrase in phrases:
        region, _score = region_store.find_by_alias(phrase)
        if region is None:
            return ParseResult(text, error=UNKNOWN_REGION, error_detail=phrase)
        targets.append(region)
    return ParseResult(text, targets=targets)


def build_command_phrases(region_store):
    """Danh sach cau lenh mau (co dau) tu cac bi danh trong region_store, dung lam
    tu vung gioi han cho Vosk va de tinh diem khop cho Google."""
    names = []
    for region in region_store.regions.values():
        names.append(region.name)
        names.extend(region.aliases)
    names = sorted(set(names))

    phrases = []
    for name in names:
        phrases.append(f'đi đến {name}')
        phrases.append(f'đi tới {name}')
        phrases.append(f'đến {name}')
        phrases.append(f'di chuyển đến {name}')
    for origin in names:
        for dest in names:
            if origin != dest:
                phrases.append(f'từ {origin} đến {dest}')
    return sorted(set(phrases))
