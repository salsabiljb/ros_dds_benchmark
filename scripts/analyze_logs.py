import os
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

def summarize(df: pd.DataFrame):
    lat = df["latency"]
    dropped = (df["seq"].diff() > 1).sum()
    stats = {
        "frames"   : len(df),
        "drops"    : int(dropped),
        "mean (ms)": lat.mean() * 1e3,
        "median"   : lat.median()*1e3,
        "p95"      : lat.quantile(0.95)*1e3,
        "p99"      : lat.quantile(0.99)*1e3,
        "std dev"  : lat.std()*1e3,
        "max"      : lat.max()*1e3,
        "min"      : lat.min()*1e3,
    }
    return stats

def plot_latency(df: pd.DataFrame, out_prefix: Path, title: str):
    lat_ms = df["latency"] * 1e3

    series_path = out_prefix.with_name(out_prefix.name + "_series.png")
    hist_path = out_prefix.with_name(out_prefix.name + "_hist.png")

    plt.figure()
    plt.plot(lat_ms.values, linewidth=0.8)
    plt.xlabel("Frame index")
    plt.ylabel("Latency (ms)")
    plt.title(f"Latency over time\n{title}")
    plt.tight_layout()
    plt.savefig(series_path)
    plt.close()

    plt.figure()
    plt.hist(lat_ms, bins=50, density=True, alpha=0.6)
    lat_ms.plot(kind="kde")
    plt.xlabel("Latency (ms)")
    plt.ylabel("Density")
    plt.title(f"Latency distribution\n{title}")
    plt.tight_layout()
    plt.savefig(hist_path)
    plt.close()

def analyze_logs(csv_files):
    os.makedirs("performance_report", exist_ok=True)

    for csv_path in csv_files:
        df = pd.read_csv(csv_path)
        stats = summarize(df)
        bn = Path(csv_path).stem
        print(f"\n=== {bn} ===")
        for k, v in stats.items():
            print(f"{k:10}: {v:>8.2f}" if "ms" in k or "std" in k or k in ("max","min")
                  else f"{k:10}: {v}")

        plot_latency(df,
                     Path("performance_report")/bn,
                     title=bn)


csv_files = [str(p) for p in Path("logs").glob("*.csv")]
analyze_logs(csv_files)

