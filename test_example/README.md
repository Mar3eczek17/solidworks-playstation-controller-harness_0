# SolidWorks RL Environment — PlayStation Controller

A test RL environment containing a single SolidWorks CAD task: `SolidWorks/1_playstation_controller`. The goal of this repo is to **update — and potentially completely rewrite — the currently AI-generated grading harness** for the task: the script that scores a candidate `.SLDPRT` part against the task's rubric. The current harness at `tests/task/harness/harness.py` is the starting point, not a fixed reference; treat it as replaceable so long as the grading contract below is honored.

## Deliverable

A grading harness for `SolidWorks/1_playstation_controller`, implemented appropriately for the task, that:

- **passes the reference**: `solution/solution.SLDPRT` scores full marks;
- **fails the adversarials**: every `.SLDPRT` under `examples/` loses points on the component(s) it actually gets wrong, and only those.

"Implemented appropriately" means the harness grades the geometry the instruction asks for, not names, feature-tree structure, or coincidences of the reference file, so that a candidate who solves the task a different way still scores full marks. Include a short note on what you changed and why, plus the score envelopes from running it over the reference and every example if you have access to SolidWorks.

## First step: fetch the assets

The (very) large `.SLDPRT` files (input part, solution, examples) live in Azure Blob Storage, not git — the checkout only has their URLs and checksums in `task.toml`. Before doing anything else, run:

```bash
pip install -r env_requirements.txt   # pywin32 installs on Windows only
python3 tools/fetch.py
```

This downloads every missing or checksum-mismatched asset into place. Nothing in this repo works without it.

## The task

See `SolidWorks/1_playstation_controller/instruction.md` for the exact prompt.

## Layout of a task

Inside `SolidWorks/1_playstation_controller/`:

| Path | Purpose |
|---|---|
| `instruction.md` | The prompt. |
| `task.toml` | Task definition: metadata, scoring components, asset URLs/checksums, environment config. |
| `environment/` | The starting part (`input.SLDPRT`) and a (non-runnable, see below) Dockerfile. |
| `solution/` | The reference answer (`solution.SLDPRT`) — the "right" edit, which should score full marks. |
| `examples/` | Candidate parts to **test the harness against**, one subfolder per example (`examples/<name>/<name>.SLDPRT`, with an `.STL` mesh and `.png` render beside it): mostly adversarial near-misses (widened but clusters unmoved, 30 mm instead of 15, only one cluster moved, missing glyphs, an unrequested change elsewhere, rebuild-error-riddled trees, …). |
| `tests/` | The harness to update/rewrite (`tests/task/harness/harness.py`), along with frozen baseline measurements of the input part (`tests/task/prompt/input.json`) and the verifier entrypoint (`test.sh`). |

Shared measurement/capture/scoring code used by harnesses lives in `common/` (`solidworks_measure.py`, `solidworks_capture.py`, `harness_base.py`, …).

## The harness

The harness grades **geometry only** (mass properties, centroids, face areas), never feature or body names — candidates may remodel freely. It scores a set of named components, each weighted, and prints a JSON score envelope (`{"score", "max_score", "passed", "subscores"}`; see `common/harness_base.py`). The components and their weights live in the harness and are yours to edit: split, merge, reweight, or add them as the rubric needs, keeping `max_score` in `task.toml` equal to their sum.

To validate a harness, run it over `solution/solution.SLDPRT` (should score full marks) and every `.SLDPRT` under `examples/`.

## Running it — Windows + SolidWorks required

SolidWorks cannot run headlessly or in a container: the harness drives a live, licensed SolidWorks session on Windows via COM (`pywin32`). Run it locally on a Windows machine with SolidWorks installed (you can also use WSL):

```bat
cd SolidWorks\1_playstation_controller
python tests\task\harness\harness.py solution\solution.SLDPRT
python tests\task\harness\harness.py examples\adversarial_widened_by_30mm\adversarial_widened_by_30mm.SLDPRT
```
