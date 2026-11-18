# Step-by-Step: Push to GitHub

**You already have git installed**: your `LunarProject` folder has 7 commits in it. So skip installation and start at Step 1.

Open **Terminal** (press `Cmd + Space`, type `Terminal`, hit Enter). Every command below gets pasted there, one at a time, pressing Enter after each.

> **Rule for this whole guide:** if a command prints an error, stop and read it before running the next one. Don't push through. The Troubleshooting section at the bottom covers the common ones.

---

## Step 0: Confirm git works (10 seconds)

```bash
git --version
```

Expect something like `git version 2.39.5`. Any version is fine.

<details>
<summary>If you get "command not found"</summary>

Run `xcode-select --install`, click **Install** in the popup, wait ~5 minutes, then re-run `git --version`.
</details>

---

## Step 1: Tell git who you are

Only needed once per computer. If you've committed before, this is already set: running it again is harmless.

```bash
git config --global user.name "Kenneth Zhou"
git config --global user.email "zhoukenneth814@gmail.com"
```

> Use the **same email** as your GitHub account, or your commits won't link to your profile.

---

## Step 2: Create a GitHub account

Skip if you have one.

1. Go to **https://github.com/signup**
2. Sign up with `zhoukenneth814@gmail.com`
3. Verify your email

**Then turn on two-factor authentication**: GitHub requires it for all contributors, and it's easier to do now than mid-push. Profile photo (top-right) → Settings → Password and authentication → Two-factor authentication.

---

## Step 3: Create the empty repository

1. Go to **https://github.com/new**
2. Fill in:

| Field | Value |
|---|---|
| Repository name | `lunar-landing-analysis` |
| Description | `MuJoCo Monte Carlo landings on procedural lunar terrain, with checks on the recommended site` |
| Visibility | **Public** |
| Add a README file |  **leave unchecked** |
| Add .gitignore |  **None** |
| Choose a license |  **None** |

>  **Those last three must stay empty.** You already have a README, .gitignore, and LICENSE. If GitHub creates its own, your first push gets rejected with a confusing "unrelated histories" error.

3. Click **Create repository**
4. Leave the page open: you'll copy the URL from it in Step 6

---

## Step 4: Placeholders (already done)

Your username `kkennethzhou` is already filled into `pyproject.toml`, `README.md`, and `CONTRIBUTING.md`. Nothing to do here.

Confirm if you like:

```bash
cd ~/Downloads/MyFiles/LunarProject/github-release
grep -rn "CHANGEME\|YOUR-USERNAME" . --include="*.md" --include="*.toml"
```

**No output means everything is set.**

---

## Step 5: Initialize the repo and commit

```bash
cd ~/Downloads/MyFiles/LunarProject/github-release
git init
git add -A
```

### Check what you're about to commit: do not skip this

```bash
git status --short | wc -l
```

**You should see a number between 20 and 30.** If it's in the hundreds or thousands, `.gitignore` isn't working: jump to Troubleshooting.

Now confirm no giant files snuck in:

```bash
git count-objects -vH | grep size
```

**Should be well under 1 MB.** If it says hundreds of MB, stop: see Troubleshooting.

One more check, that the NASA images are properly excluded:

```bash
git status --short | grep -i "lunar_images\|\.pptx\|\.MOV" || echo "clean - no large files staged"
```

You want to see `clean - no large files staged`.

### Commit

```bash
git commit -m "Initial release: MuJoCo lunar landing analysis with validation suite

Monte Carlo landing site selection pipeline. Published as work in progress:
the validation suite currently fails its signal and bootstrap checks, and two
physics defects are documented but unfixed. See docs/known-issues.md."
```

---

## Step 6: Connect to GitHub and push

```bash
git branch -M main
git remote add origin https://github.com/kkennethzhou/lunar-landing-analysis.git
git push -u origin main
```

That URL is already correct for your account: paste the three lines as-is.

### When it asks for a password

A browser window or a terminal prompt will appear.

**If a browser opens:** click **Authorize**. Done.

**If the terminal asks for a username and password:** your GitHub account password will **not** work. GitHub removed password authentication in 2021. You need a Personal Access Token:

1. Go to **https://github.com/settings/tokens/new**
2. Note: `lunar-landing-analysis push`
3. Expiration: 90 days
4. Check the **`repo`** box (the top-level one: it selects the sub-items automatically)
5. Scroll down, click **Generate token**
6. **Copy the token now**: `ghp_...`. GitHub shows it exactly once.
7. Back in Terminal: username is your GitHub username, and **paste the token as the password**

> The cursor won't move while you paste the password. That's normal: it's hiding your input. Paste and press Enter.

<details>
<summary>Save the token so you only do this once</summary>

```bash
git config --global credential.helper osxkeychain
```

Run this *before* pushing, and macOS Keychain stores the token after the first successful push.
</details>

---

## Step 7: Verify

```bash
open https://github.com/kkennethzhou/lunar-landing-analysis
```

Check that:

- [ ] The README renders with the badges at top
- [ ] There's **no** `lunar_images` folder
- [ ] Repo size is small: Settings → scroll down, should be a few hundred KB
- [ ] `docs/`, `src/`, `tests/`, `scripts/` are all there

---

## Step 8: Polish the repo page

**Add topics**: click the  gear next to "About" on the right side:

```
mujoco  simulation  monte-carlo  lunar  nasa  robotics  physics-simulation
```

**Open issues for the known defects.** Issues tab → New issue. Three worth creating, copying the relevant section from `docs/known-issues.md` into each:

| Title | Label |
|---|---|
| BLOCKING-1: Landing gear cannot support vehicle at rest (k=320 N/m) | `bug` |
| BLOCKING-2: Thrust never cut at touchdown | `bug` |
| OPEN-1: No terrain signal in safety map: is it recoverable? | `help wanted` |

Pin OPEN-1 (⋯ menu on the issue → Pin issue). It's the most interesting problem in the repo and the one most likely to attract a good contributor.

---

## Making changes later

The everyday loop, once the repo exists:

```bash
cd ~/Downloads/MyFiles/LunarProject/github-release

git status                        # what changed?
git add -A                        # stage everything
git commit -m "Fix leg stiffness: 320 -> 3000 N/m"
git push
```

That's it: after the first push, plain `git push` works with no extra arguments.

Useful when things go sideways:

```bash
git log --oneline                 # commit history
git diff                          # unstaged changes
git restore FILENAME              # undo changes to one file
git reset HEAD~1                  # undo last commit, keep the edits
```

---

## Troubleshooting

### `git status` shows thousands of files

`.gitignore` isn't being applied. Almost always because files were staged before it existed:

```bash
cd ~/Downloads/MyFiles/LunarProject/github-release
git rm -r --cached .
git add -A
git status --short | wc -l        # should now be 20-30
```

### Push rejected: "remote contains work you do not have locally"

You checked one of the "Add README / .gitignore / license" boxes in Step 3. Easiest fix: delete the GitHub repo (Settings → scroll to bottom → Delete this repository) and redo Step 3 with all three boxes unchecked.

### "Support for password authentication was removed"

Expected. Use a Personal Access Token: see Step 6.

### "remote origin already exists"

You ran `git remote add` twice:

```bash
git remote set-url origin https://github.com/kkennethzhou/lunar-landing-analysis.git
```

### Push hangs or is very slow

Large files are being uploaded. Cancel with `Ctrl+C` and check:

```bash
git count-objects -vH | grep size-pack
```

Over ~10 MB means something big got in. Find it:

```bash
git ls-files | xargs -I{} du -h {} 2>/dev/null | sort -rh | head -10
```

### "failed to push some refs"

```bash
git pull --rebase origin main
git push
```

### I want to start completely over

Nothing is lost: this deletes only git's bookkeeping, not your files:

```bash
cd ~/Downloads/MyFiles/LunarProject/github-release
rm -rf .git
```

Then start again from Step 5.

---

## What about the old 1.9 GB repo?

Your `LunarProject/.git` has the NASA PNGs committed, which is why it's 1.9 GB. **Leave it alone**: it's your private working history and it's useful. It just shouldn't go to GitHub.

`github-release/` is a completely separate repo with its own `.git`. The two don't interact.

If you ever want the phase-by-phase commit history public too, that needs `git-filter-repo` to strip the images from history first: a separate job, and not necessary for a first release.
