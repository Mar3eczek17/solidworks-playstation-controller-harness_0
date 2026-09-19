#!/bin/bash
# TODO: not runnable in a container. tests/task/harness/harness.py grades
# the SolidWorks document that is currently open and active via COM
# (win32com), or a path passed as argv[1] that it loads into a live
# SolidWorks session -- either way it needs a real, licensed, GUI SolidWorks
# process on Windows, which this environment cannot provide (see
# environment/Dockerfile).
#
# Run locally on Windows instead, from this task's directory:
#   python tests/task/harness/harness.py <path-to-candidate.SLDPRT>
#   # or, to grade whatever is currently open in SolidWorks:
#   python tests/task/harness/harness.py
#
# It prints a JSON score envelope to stdout (see common/harness_base.py's
# finalize()): {"score": ..., "max_score": 4, "passed": ..., "subscores": {...}}
# Components are continuous (0..1) and weighted to sum to 4.0:
#   widened by 15 mm                 w=1.0
#   clusters at mirrored positions   w=1.0
#   left-handed layout achieved      w=1.0
#   no new control interference      w=0.5
#   no unrequested changes           w=0.25
#   glyphs preserved                 w=0.25
set -euo pipefail
echo "TODO: no automated verifier here -- see comments in this file" >&2
mkdir -p /logs/verifier 2>/dev/null || true
exit 1
