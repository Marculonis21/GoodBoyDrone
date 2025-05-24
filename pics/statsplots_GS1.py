#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os
import seaborn as sns

# pandas.read_csv()

files = sorted([x for x in os.listdir("CosyneGS1") if x.startswith("gsCoSyNE128")])

dfs = []
for file in files:
    df = pd.read_csv(f"CosyneGS1/{file}", header=None, names=["gen", "max", "min", "avg", "med", "none"])

    df["mprob"] = file.split("_")[1]
    df["mcauchy"] = file.split("_")[2]
    df["run"] = file.split("_")[-1][3]
    dfs.append(df)

all_data = pd.concat(dfs, ignore_index=True)

group_cols = ["gen","mprob","mcauchy"]
aggregated = all_data.groupby(group_cols).agg(
    avg_mean = ("avg", "mean"),
).reset_index()

# sns.lineplot(data=aggregated, x="gen", y="max_mean", hue="mprob", style="mcauchy")
sns.lineplot(data=aggregated, x="gen", y="avg_mean", hue="mprob", style="mcauchy", errorbar=None)
plt.xlabel("Generation")
plt.legend()
plt.tight_layout()
plt.show()
