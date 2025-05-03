#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os
import seaborn as sns

# pandas.read_csv()

files = sorted([x for x in os.listdir("./alg_eval/") if x.startswith("eval_")])

dfs = []
for file in files:
    df = pd.read_csv(f"alg_eval/{file}", header=None, names=["gen", "max", "min", "avg", "med", "none"])

    df["alg"] = file.split("_")[1]
    df["popSize"] = file.split("_")[2]
    df["run"] = file.split("_")[-1][3]
    dfs.append(df)

all_data = pd.concat(dfs, ignore_index=True)

group_cols = ["gen","alg","popSize"]
aggregated = all_data.groupby(group_cols).agg(
    max_mean = ("max", "mean"),
    avg_mean = ("avg", "mean"),
).reset_index()

# sns.lineplot(data=aggregated[aggregated["alg"] == "cosyne"], x="gen", y="max_mean", hue="alg", style="popSize", errorbar=None)
# sns.lineplot(data=aggregated[aggregated["alg"] == "cosyne"], x="gen", y="avg_mean", hue="alg", style="popSize", errorbar=None)

sns.lineplot(data=aggregated, x="gen", y="max_mean", hue="alg", style="popSize", errorbar=None)
# sns.lineplot(data=aggregated, x="gen", y="avg_mean", hue="alg", style="popSize", errorbar=None)
plt.xlabel("Generation")
plt.ylabel("Mean of Max Fitness")
plt.legend()
plt.tight_layout()
plt.show()
