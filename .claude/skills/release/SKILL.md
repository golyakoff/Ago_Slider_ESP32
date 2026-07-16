---
name: release
description: Publish a firmware release - update changelog, tag vX.Y.Z, push, and monitor the GitHub Actions build. Use when asked to release, publish, or tag a new firmware version.
---

# Release the firmware

Releases are built by `.github/workflows/release.yml` on a `v*.*.*` tag push: two
variants (debug / release) from `sdkconfig.defaults` + overlays, published as
`ago_slider_vX.Y.Z_{debug,release}_16mb_{fw,pt,bl,od}.bin` with generated esptool
instructions. The Android app's update check downloads the asset ending in
`_release_16mb_fw.bin` and compares the release tag with the device version.

## Pre-flight checks

1. Working tree must be clean (do not sweep in the user's long-standing local
   `sdkconfig` / `.vscode/settings.json` modifications — ask if they're still pending).
2. If `sdkconfig` changed since `sdkconfig.defaults` was last regenerated, regenerate:
   activate the IDF env (see the `flash` skill) and run `idf.py save-defconfig`,
   review the diff, commit. CI builds from the defaults, not from `sdkconfig`.
3. Local build sanity: `idf.py build` must pass.
4. Firmware version is automatic: `git describe --tags` (PROJECT_VER intentionally
   unset), so a build from the tag reports exactly the tag name. No version bump in
   code is needed.

## Release steps

1. Add a section to `CHANGELOG.md` — heading format must be `# Release X.Y.Z`
   (without `v`), followed by bullet lines.
2. Commit and push `master`.
3. Tag and push (git author must match `c:\git.txt` — Andrey Golyakov
   <golyakoff@yandex.ru>; verify with `git config user.name` / `user.email`):
   ```bash
   git tag -a vX.Y.Z -m "Release vX.Y.Z"
   git push origin master vX.Y.Z
   ```

## Monitor CI

The repo is private and `gh` CLI is not installed. Use the GitHub API with the token
from the git credential manager:

```bash
TOKEN=$(printf "protocol=https\nhost=github.com\n" | git credential fill | grep "^password=" | cut -d= -f2)
curl -s -H "Authorization: token $TOKEN" \
  "https://api.github.com/repos/golyakoff/Ago_Slider_ESP32/actions/runs?per_page=3" \
  | python -c "import json,sys; [print(r['id'], r['head_branch'], r['status'], r['conclusion']) for r in json.load(sys.stdin)['workflow_runs']]"
```

The two-variant Docker build takes ~10–20 minutes. On failure, fetch the job log:
runs/<id>/jobs → jobs/<job_id>/logs (same auth header). Verify at the end that the
release exists and carries all 8 assets (`releases/tags/vX.Y.Z`).

## Re-running a failed release

If the workflow failed *before* creating the release, it is safe to fix, commit, and
move the tag to the new commit:

```bash
git tag -d vX.Y.Z && git push origin :refs/tags/vX.Y.Z
git tag -a vX.Y.Z -m "Release vX.Y.Z" && git push origin vX.Y.Z
```

If a release already exists, do NOT move the tag — bump the patch version instead.

## Known trap

Linux CI is case-sensitive; Windows is not. If CI fails with "Cannot find source
file" while the local build passes, check filename case vs `CMakeLists.txt` SRCS and
`#include` directives (all component files are normalized to lowercase).
