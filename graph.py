import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("experiment_scatter_log.csv")
fig, ax = plt.subplots(figsize=(8, 3))
ax.plot(df.time_s, df.active_groups, label="Active groups")
ax.plot(df.time_s, df.satellite_groups, label="Satellite groups")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Group count")
ax.legend()
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig("figB4_group_dynamics.png", dpi=200)