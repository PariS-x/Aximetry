# Putting Aximetry on GitHub

## 1. Create the repository

On github.com, click **New repository**, name it `aximetry`, set it to Public, and
**leave "Add a README" unticked** (the project already has one).

## 2. Push from your computer

In a terminal, from inside the `aximetry` folder:

```bash
git init
git add .
git commit -m "Aximetry: Mars rover design and autonomy simulator"
git branch -M main
git remote add origin https://github.com/<your-username>/aximetry.git
git push -u origin main
```

When git asks for a password, use a **personal access token** (GitHub → Settings →
Developer settings → Personal access tokens), not your account password.

**Shortcut with the GitHub CLI** (`gh`), which creates the repo and pushes in one go:

```bash
gh auth login
git init && git add . && git commit -m "Aximetry: Mars rover design and autonomy simulator"
gh repo create aximetry --public --source=. --push
```

`build/` and generated replay pages are already in `.gitignore`.

## 3. Continuous integration

`.github/workflows/ci.yml` builds and runs the test suite on Linux, macOS and
Windows on every push. After the first push, the **Actions** tab shows the result;
a green check on the repo page tells reviewers the code builds and passes its tests.

## 4. The demo replay

GitHub shows `.html` files as source code, not as a page, so `docs/demo.html`
needs to be downloaded and opened to view it. For a clickable link, put a copy
in The Last Archive's repository (for example `archive/aximetry-demo.html`); it
is one self-contained file. A project repo can also have its own Pages site
(Settings → Pages → `main` / `docs`) without affecting the site in the
`<username>.github.io` repository; it appears under that site's address at
`/aximetry/`.

## 5. Polish

* Add a short description and topics on the repo page (robotics, mars-rover,
  terramechanics, path-planning, kalman-filter, cpp17).
* Pin the repo on your GitHub profile.
