import urllib.request, json

r = urllib.request.urlopen('http://127.0.0.1:5050/api/mission1/timeseries')
d = json.loads(r.read())

ns, np, nr, ng = 0, 0, 0, 0
for rec in d:
    m = (rec.get('MODE') or '').upper()
    if 'REGEN' in m:           nr += 1
    elif 'TAKEOFF' in m or 'PARALLEL' in m: np += 1
    elif 'SERIES' in m or 'LOITER' in m or 'GLIDE' in m: ns += 1
    else:                      ng += 1  # ground phases

tot_flight = ns + np + nr
total = tot_flight + ng

print(f"Total records  : {len(d)}")
print(f"Ground (taxi)  : {ng} ({ng/len(d)*100:.1f}%)")
print(f"-- Flight mode breakdown --")
print(f"SERIES (flight): {ns} ({ns/tot_flight*100:.1f}% of flight)")
print(f"PARALLEL       : {np} ({np/tot_flight*100:.1f}% of flight)")
print(f"REGEN          : {nr} ({nr/tot_flight*100:.1f}% of flight)")
print()
print("Architecture bars will show:")
print(f"  SERIES   : {round(ns/tot_flight*100)}%")
print(f"  PARALLEL : {round(np/tot_flight*100)}%")
print(f"  REGEN    : {round(nr/tot_flight*100)}%")

# Also show mode distribution
from collections import Counter
modes = Counter(rec.get('MODE','?') for rec in d)
print()
print("Mode counts:")
for m,c in sorted(modes.items(), key=lambda x:-x[1]):
    print(f"  {m:<25} : {c:5d} ({c/len(d)*100:.1f}%)")
