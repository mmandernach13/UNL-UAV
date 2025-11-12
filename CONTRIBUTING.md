# Contributing Guide for UAV Simulation Repo

Welcome! This document explains how to work with this repository so everyone can contribute cleanly and without conflicts.

---

## 🚀 Workflow Overview

We use a **feature-branch workflow** with **Pull Requests (PRs)** for all changes.  
Do **not** push directly to `main`.

```
main         # Always working and deployable
develop      # Optional integration branch (use if many PRs at once)
feature/*    # Individual work branches
hotfix/*     # Quick fixes to main
```

---

## 🛠️ Setting Up Your Local Repo

1. **Clone the repo**
   ```bash
   git clone git@github.com:<org>/<repo>.git
   cd <repo>
   ```

2. **Pull the latest changes**
   ```bash
   git checkout main
   git pull origin main
   ```

3. **Create a feature branch**
   ```bash
   git checkout -b feature/<short-description>
   ```
   Example:
   ```bash
   git checkout -b feature/add-sitl-autostart
   ```

---

## 🧩 Making Changes

- Keep commits small and meaningful.
- Include a clear message:
  ```bash
  git commit -m "Add script to auto-start PX4 SITL without GCS"
  ```
- Test your code before committing.

---

## 🧪 Testing Locally

1. Build your workspace:
   ```bash
   colcon build --symlink-install
   ```
2. Source your setup:
   ```bash
   source install/setup.bash
   ```
3. Run your launch scripts or tests as needed.

---

## 🔁 Keeping Your Branch Updated

Before merging:
```bash
git fetch origin
git pull origin main --rebase
```
If there are merge conflicts, fix them locally before pushing again.

---

## 💬 Submitting a Pull Request (PR)

1. Push your branch:
   ```bash
   git push origin feature/<branch-name>
   ```
2. On GitHub, open a **Pull Request** from your branch → `main`.
3. Fill out the PR template with:
   - **What you changed**
   - **Why you changed it**
   - **How to test it**
4. Wait for **review and approval**.

> PRs require at least one reviewer’s approval before merge.

---

## ⚖️ Code Review Guidelines

When reviewing a teammate’s PR:
- Check for correctness and clarity.
- Confirm that launch scripts and configs still work.
- Leave constructive comments.
- Approve once satisfied.

---

## 🧱 Branch Protection Rules (for Maintainers)

In GitHub → Settings → Branches → “main”:
- ✅ Require pull request before merging  
- ✅ Require 1 approval  
- ✅ Disallow force pushes  
- ✅ Require status checks (optional if you add CI later)

---

## 🔥 Hotfixes

If something breaks in `main`, create a hotfix branch:
```bash
git checkout -b hotfix/fix-broken-launch
# Make fix
git commit -m "Fix broken PX4 launch path"
git push origin hotfix/fix-broken-launch
```
Then make a PR and merge once approved.

---

## 🧹 Cleaning Up

After your PR is merged:
```bash
git checkout main
git pull origin main
git branch -d feature/<your-branch>
```

---

## 🤝 Good Practices

- Keep scripts portable (no user-specific paths)
- Use relative paths (`$REPO_ROOT`) when possible
- Avoid modifying the PX4 or Gazebo repos directly
- Use `px4_config/` for local configuration

---

Thanks for helping keep this repo clean and collaborative!
