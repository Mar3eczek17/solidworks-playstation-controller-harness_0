#!/bin/bash
# TODO: not automatable here. The reference answer is solution.SLDPRT (the
# engineer's widen + left-hand conversion, scores 5/5), but there is no
# headless way to "apply" a SolidWorks part inside a container -- SolidWorks
# has to actually open and rebuild the file through its own GUI/COM session.
#
# Locally on Windows this would look something like:
#   1. Open solution.SLDPRT in SolidWorks (or apply the edit by hand
#      starting from environment/input.SLDPRT)
#   2. Save as solution.SLDPRT
#   3. Run tests/task/harness/harness.py against it (or against the active
#      document)
set -euo pipefail
echo "TODO: no automated oracle -- see comments in this file" >&2
exit 1
