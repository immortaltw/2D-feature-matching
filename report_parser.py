import pandas as pd
import os
import sys

def parse():
    path = "report/"
    filelist = os.listdir("report/")
    result = []
    for fp in filelist:
        if fp.endswith(".csv"):
            stats = pd.read_csv(path + fp, delimiter=',')
            result.append((os.path.splitext(fp)[0], stats["detectTime"].mean(), stats["computeTime"].mean()))

    res_df = pd.DataFrame(sorted(result, key=lambda obj: obj[1]+obj[2]), columns = ["combinations", "avg detectTime", "avg computeTime"])
    res_df["feature extraction time"] = res_df["avg detectTime"] + res_df["avg computeTime"]
    print(res_df)
    res_df.to_csv("final_report.csv", index=None)

if __name__ == "__main__":
    parse()