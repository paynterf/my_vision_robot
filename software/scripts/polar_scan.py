#!/usr/bin/env python3
"""
Polar plot of a WallE RunToDaylight scan.
Automatically finds the widest far run (including wrap-around)
and highlights it.
"""

import re
import numpy as np
import matplotlib.pyplot as plt

raw = """
MSec	FrontD	Hdg
8712	26.9	  0.1
8975	28.3	 -7.1
9175	35.4	 -11.3
9439	37.4	 -17.9
9703	40.4	 -23.9
9790	43.6	 -27.8
10053	44.7	 -34.7
10262	47.9	 -39.0
10525	54.2	 -45.4
10669	64.7	 -49.3
10869	65.7	 -53.1
11069	83.0	 -57.0
11333	109.6	 -63.8
11477	143.0	 -67.8
11804	140.6	 -80.3
11892	120.8	 -85.9
12155	118.0	 -92.5
12299	106.0	 -96.7
12508	46.2	 -100.8
12651	35.6	 -104.9
12851	40.0	 -108.9
13115	47.6	 -115.5
13315	56.1	 -119.8
13515	54.5	 -123.7
13842	51.6	 -129.9
13930	48.5	 -134.5
14193	45.5	 -141.6
14402	43.1	 -145.9
14665	41.6	 -152.7
14865	40.6	 -156.8
15129	41.0	 -163.3
15209	43.5	 -167.2
15473	82.1	 -172.8
15735	417.9	 -177.3
15879	398.0	  179.0
16143	400.2	  172.6
16407	389.8	  166.7
16671	337.7	  160.6
16871	299.6	  156.5
17135	280.4	  149.9
17399	267.7	  143.6
17606	244.1	  139.4
17869	237.0	  133.3
18078	229.1	  129.0
18341	228.3	  122.3
18541	91.9	  118.0
18805	90.3	  111.1
19069	90.7	  104.5
19213	93.9	  100.5
19420	94.2	  96.5
19563	87.8	  92.2
19772	83.3	  88.1
19915	71.0	  84.3
20115	69.5	  80.7
20315	62.1	  76.8
20579	57.8	  70.1
20779	48.6	  66.0
20979	47.4	  62.2
21186	45.3	  58.2
21449	42.4	  53.0
21538	40.2	  48.8
21746	39.2	  44.9
21945	38.4	  40.8
22209	37.7	  34.2
22409	34.4	  30.2
22609	34.4	  26.2
22873	33.1	  20.8
23080	37.5	  17.1
23343	38.0	  11.3
23432	39.2	  7.2
23640	40.0	  3.4
23783	43.9	 -0.9
"""

THRESHOLD = 200.0

# --- parse -----------------------------------------------------------------
rows = []
for line in raw.strip().splitlines():
    line = line.strip()
    if not line or "FrontD" in line:
        continue
    nums = re.findall(r"-?\d+\.?\d*", line.replace(" - ", " -"))
    if len(nums) >= 3:
        dist = float(nums[1])
        hdg  = float(nums[2])
        rows.append((hdg, dist))

hdg  = np.array([r[0] for r in rows])
dist = np.array([r[1] for r in rows])

def terminate_run(i):
    global bestWidth, bestStart, bestEnd, numruns, runStart
    width = hdg[i-1] - hdg[runStart]
    if width > 180:  width -= 360
    if width < -180: width += 360
    width = abs(width)
    if width > bestWidth:
        bestWidth = width
        bestStart = runStart
        bestEnd   = i - 1
    runStart = -1
    numruns += 1



N = len(hdg)
print(f"Parsed {N} samples")

# --- GetBestDistance (ported from RobotBestDistV2.cpp) --------------------
bestStart = -1
bestEnd   = -1
bestWidth = 0.0
numruns   = 0
runStart  = -1


for i in range(N):
    if dist[i] >= THRESHOLD:
        if runStart < 0:
            runStart = i
        if i >= N - 1:
            terminate_run(i + 1)   # close run that reaches the end
    elif runStart >= 0:
        terminate_run(i)

# wrap-around stitch
if dist[0] >= THRESHOLD and dist[N-1] >= THRESHOLD:
    endRun = N - 1
    while endRun > 0 and dist[endRun] >= THRESHOLD:
        endRun -= 1
    endRun += 1

    startRun = 0
    while startRun < N - 1 and dist[startRun] >= THRESHOLD:
        startRun += 1
    startRun -= 1

    width = hdg[startRun] - hdg[endRun]
    if width > 180:  width -= 360
    if width < -180: width += 360
    width = abs(width)

    if width > bestWidth:
        bestWidth = width
        bestStart = endRun
        bestEnd   = startRun
        numruns -= 1

print(f"numruns = {numruns}, bestwidth = {bestWidth:.1f}, "
      f"bestStart = {bestStart}, bestEnd = {bestEnd}")

# --- build win_idx from the computed bestStart/bestEnd --------------------
if bestStart <= bestEnd:
    win_idx = list(range(bestStart, bestEnd + 1))
else:
    # wrap-around
    win_idx = list(range(bestStart, N)) + list(range(0, bestEnd + 1))

    
bestHdg_idx = win_idx[len(win_idx) // 2]
print(f"bestHdg_idx = {bestHdg_idx}, bestHdg = {hdg[bestHdg_idx]:.1f}, bestDist = {dist[bestHdg_idx]:.1f}")

    

# --- polar plot ------------------------------------------------------------
order = np.argsort(hdg)
theta_sorted = np.deg2rad(hdg[order])
dist_sorted  = dist[order]

fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection="polar")
ax.plot(theta_sorted, dist_sorted, "o", markersize=5, label="FrontD")

if win_idx:
    ax.plot(np.deg2rad(hdg[win_idx]), dist[win_idx], "o",
            color="green", markersize=7, label="best run")
    ax.plot(np.deg2rad(hdg[bestHdg_idx ]), dist[bestHdg_idx], "o",
            color="red", markersize=10, label="best hdg/dist")

ax.set_theta_zero_location("N")
ax.set_theta_direction(-1)
tick_angles = np.arange(0, 360, 45)
tick_labels = [f"{((a + 180) % 360) - 180}°" for a in tick_angles]
ax.set_thetagrids(tick_angles, tick_labels)
ax.set_title("261019 RunToDaylight scan with robot on the floor")
ax.legend(loc="upper right", bbox_to_anchor=(1.3, 1.1))

# bottom label with chosen heading and distance
chosen_hdg = hdg[bestHdg_idx]
chosen_dist = dist[bestHdg_idx]
chosen_str = f"Best Heading and Direction: {chosen_hdg:.1f}°   {chosen_dist:.0f} cm"
chosen_strlen = len(chosen_str)

plt.subplots_adjust(bottom=0.12)
#ax.text(0.5, -0.16,
#ax.text(0.5, 0,
ax.text(0.5, -0.08,
        f"Best Heading and Distance: {chosen_hdg:.1f}°   {chosen_dist:.0f} cm",
        transform=ax.transAxes,
        ha="center", va="top", fontsize=12,
        clip_on=False)
        
plt.tight_layout()
plt.show()