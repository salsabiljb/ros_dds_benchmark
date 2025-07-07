import pandas as pd
import sys

csv_path = sys.argv[1]
df = pd.read_csv(csv_path)
latencies = df['latency']

print(f"Average Latency: {latencies.mean()*1000:.2f} ms")
print(f"Dropped Frames: {df['seq'].diff().gt(1).sum()}")
print(f"Latency Std Dev: {latencies.std()*1000:.2f} ms")
