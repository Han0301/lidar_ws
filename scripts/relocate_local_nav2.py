"""Repair missing absolute export paths only inside an extracted Nav2 prefix."""
import argparse,re
from pathlib import Path
parser=argparse.ArgumentParser();parser.add_argument('prefix',type=Path);args=parser.parse_args();prefix=args.prefix.resolve()
for file in prefix.glob('share/*/cmake/*.cmake'):
    source=file.read_text()
    def relocate(match):
        original=Path(match.group());candidate=prefix/original.relative_to('/opt/ros/jazzy')
        return str(candidate) if not original.exists() and candidate.exists() else match.group()
    updated=re.sub(r'(?<![\w/.\-])/opt/ros/jazzy/[^;"\s]+',relocate,source)
    if updated!=source:file.write_text(updated)
