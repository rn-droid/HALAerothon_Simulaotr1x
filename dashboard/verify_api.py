import urllib.request, json

r = urllib.request.urlopen('http://127.0.0.1:5050/')
print('HTTP', r.status)
r2 = urllib.request.urlopen('http://127.0.0.1:5050/api/mission1/timeseries')
d = json.loads(r2.read())
print('Timeseries records:', len(d))

# Check BATTERY_POWER is negative during descent (regen)
regen_recs = [rec for rec in d if (rec.get('BATTERY_POWER') or 0) < -0.5 and (rec.get('ENGINE_POWER') or 0) < 0.5]
print('Regen records (bat<-0.5kW, eng off):', len(regen_recs))
if regen_recs:
    ex = regen_recs[0]
    t = ex['TIME']
    bp = ex['BATTERY_POWER']
    ep = ex['ENGINE_POWER']
    mo = ex['MODE']
    print('  t=', t, 's bat=', bp, 'kW eng=', ep, 'kW mode=', mo)
    # Expected regen display: -bat = positive regen power
    print('  Expected REGEN STORED:', round(-bp, 2), 'kW')

# Verify MODE field exists
modes = set(rec.get('MODE','?') for rec in d)
print('Unique modes in API data:', sorted(modes))

# Check first record (t=0) state
f = d[0]
print('First record t=0: ENGINE_POWER=', f.get('ENGINE_POWER'), 'BATTERY_SOC=', f.get('BATTERY_SOC'), 'ALTITUDE=', f.get('ALTITUDE'))
