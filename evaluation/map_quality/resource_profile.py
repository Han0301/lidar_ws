"""Read-only per-job process-tree CPU and sampled RSS, including launcher children."""
import csv
import json
import os
import threading
import time
from pathlib import Path


class ResourceProfile:
    def __init__(self, output, jobs):
        self.output, self.jobs = output, jobs
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.sample, daemon=True)
        self.totals, self.peaks = {}, {}
        self.known, self.previous = {}, {}
        self.interval = .5

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.thread.join()
        (self.output/'resources.json').write_text(json.dumps(dict(
            method='Linux /proc process trees; CPU sums per-pid maxima; RSS sums live processes every 0.5 s',
            cpu_seconds=self.totals, sampled_peak_rss_mib=self.peaks,
            sample_interval_s=self.interval, rss_limit='Sampled peak; brief peaks between samples can be missed'), indent=2))

    def sample(self):
        ticks = os.sysconf('SC_CLK_TCK')
        with (self.output/'resources.csv').open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['mono_s','job','cpu_s','cpu_one_core_percent','rss_mib','pids'])
            while not self.stop_event.is_set():
                now = time.monotonic()
                records = {}
                for path in Path('/proc').glob('[0-9]*/stat'):
                    try:
                        values = path.read_text().split(') ',1)[1].split()
                        pid = int(path.parent.name)
                        records[pid] = (int(values[1]), (int(values[11])+int(values[12]))/ticks,
                                        int(values[21])*os.sysconf('SC_PAGE_SIZE')/2**20)
                    except (OSError, ValueError, IndexError):
                        pass
                for label, process in list(self.jobs.items()):
                    selected = {process.pid} if process.pid in records else set()
                    while True:
                        extra = {pid for pid,r in records.items() if r[0] in selected}
                        if extra <= selected:
                            break
                        selected |= extra
                    known = self.known.setdefault(label,{})
                    for pid in selected:
                        known[pid] = max(known.get(pid,0),records[pid][1])
                    cpu = sum(known.values())
                    rss = sum(records[pid][2] for pid in selected)
                    old_time, old_cpu = self.previous.get(label,(now,cpu))
                    percent = 100*(cpu-old_cpu)/(now-old_time) if now>old_time else 0
                    self.previous[label] = now,cpu
                    self.totals[label] = cpu
                    self.peaks[label] = max(rss,self.peaks.get(label,0))
                    writer.writerow([now,label,cpu,percent,rss,' '.join(map(str,sorted(selected)))])
                f.flush()
                self.stop_event.wait(self.interval)
