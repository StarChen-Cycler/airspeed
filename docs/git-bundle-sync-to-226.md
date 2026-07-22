# Syncing the Repo to 226 via git bundle over SSH

Use this when 226 cannot reach GitHub (remote fetch fails) but SSH works.
`git bundle` packages commits into a single file; we transport it with `scp`
and let 226 fast-forward from it — full git history stays intact, and 226's
`origin/main` ref is updated so normal `git pull` works again once GitHub is
reachable.

Machines: local dev = 227 (`/home/zyc_airspeed/WS/uniform-robot-data-collection-3in1/airspeed`),
target = 226 (`/home/intern/ros2-test/airspeed`, user `intern`, passwordless SSH configured).

## Procedure

**1. Find the shared base commit (what 226 currently has):**

```bash
ssh intern@10.60.2.226 'git -C /home/intern/ros2-test/airspeed log --oneline -1'
```

**2. Package everything since that commit (run locally):**

```bash
cd /home/zyc_airspeed/WS/uniform-robot-data-collection-3in1/airspeed
git bundle create /tmp/airspeed-sync.bundle <BASE_COMMIT>..main
git bundle verify /tmp/airspeed-sync.bundle
```

**3. Transport:**

```bash
scp /tmp/airspeed-sync.bundle intern@10.60.2.226:/tmp/
```

**4. Apply on 226 (fast-forward only — never force):**

```bash
ssh intern@10.60.2.226 'cd /home/intern/ros2-test/airspeed && \
  git fetch /tmp/airspeed-sync.bundle main:refs/remotes/origin/main && \
  git merge --ff-only refs/remotes/origin/main && \
  git log --oneline -3'
```

**5. Verify:** `git log` shows the new HEAD and `git status` shows only 226's
known local changes (machine configs, untracked tools) — nothing else.

## Handling 226's uncommitted local changes

The merge refuses if a locally-modified file would be overwritten. 226 keeps
machine-specific edits (`config/camera.yaml`, session yaml, `solver_smooth.yaml`,
`convert_h5_to_lerobot.py`) that must NOT be discarded. Only reset files whose
changes were already ported upstream (check first):

```bash
# unstage everything (keeps working tree):
git reset -q
# discard local edits ONLY for files the bundle now owns:
git checkout HEAD -- <path/to/file>
# then re-run the merge
```

## Rollback

```bash
git -C /home/intern/ros2-test/airspeed checkout <PREVIOUS_COMMIT>   # restart services after
```

## Rules of thumb

- Code flows one direction only: **local → bundle → 226**. Never edit tracked
  source on 226; configs only.
- Always `--ff-only`. If the merge can't fast-forward, stop and inspect —
  someone committed on 226 directly.
- After the sync, 226's `origin/main` points at the new HEAD, so a later
  `git pull` (when GitHub is reachable) continues normally with no conflicts.
