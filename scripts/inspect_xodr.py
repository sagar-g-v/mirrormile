"""Quick statistics on the generated OpenDRIVE file."""
import collections
import re
import sys

import config as C

x = (open(sys.argv[1]) if len(sys.argv) > 1 else open(C.XODR_FILE)).read()
print(x[:700])
print("roads", len(re.findall(r"<road ", x)), "junctions", len(re.findall(r"<junction ", x)))
print("lane types", collections.Counter(re.findall(r'<lane id="-?\d+" type="(\w+)"', x)))
print("roadMarks", collections.Counter(re.findall(r'<roadMark [^>]*type="([^"]+)"[^>]*color="(\w+)"', x)).most_common(10))
print("geometry", collections.Counter(re.findall(r"<geometry[^>]*>\s*<(\w+)", x)))
print("signals", len(re.findall(r"<signal ", x)), "objects", len(re.findall(r"<object ", x)))
