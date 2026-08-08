import pathlib
html = pathlib.Path('dashboard/index.html').read_text(encoding='utf-8')
print('class Battery found:', 'class Battery' in html)
print('class Supercapacitor found:', 'class Supercapacitor' in html)
checks = [
    ('ENG_MAX_W: 60000.0', '60 kW engine'),
    ('BAT_CAP_WH: 36000.0', '36 kWh battery'),
    ('M_FUEL_INIT: 64.0', '64 kg fuel'),
    ('BAT_SOC_INIT: 0.90', '90% initial SOC'),
    ('BAT_SOC_MIN: 0.20', '20% SOC floor'),
    ('ETA_PROP: 0.82', 'Propeller eta=0.82'),
    ('ETA_MOTOR: 0.95', 'Motor eta=0.95'),
    ('ETA_GEN: 0.92', 'Generator eta=0.92'),
    ('BSFC_KG_KWH: 0.60', 'BSFC=0.60'),
    ('LOITER_BANK_DEG: 25.0', 'Loiter bank 25 deg'),
    ('ETA_REGEN', 'Regen path efficiency'),
    ('M1_TOTAL_S', 'Total mission time var'),
    ('ENGINE_START_TAXI', 'Phase 1 name'),
    ('LOITER_ORBITS', 'Loiter orbit tracking'),
    ('missionT >= M1_TOTAL_S', 'Mission complete check'),
    ('split_power', 'ECMS function reference in comment'),
    ('splitPower', 'splitPower function'),
    ('runSimulation', 'runSimulation function'),
    ('loadAPIData', 'API loader'),
    ('DOMContentLoaded', 'Init listener'),
]
ok=0
for pat, label in checks:
    found = pat in html
    if found: ok += 1
    else: print(f'  MISSING: [{label}] = {repr(pat[:50])}')
print(f'Physics constant checks: {ok}/{len(checks)} OK')
print(f'File: {len(html):,} bytes, {html.count(chr(10)):,} lines')
