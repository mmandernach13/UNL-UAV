# Contributing to the UNL-UAV Project

First off, thank you for considering contributing! We're excited to have you on board. This guide is designed to help you get started, even if you've never contributed to a project on GitHub before.

## The Big Picture: How We Work

We use a workflow that's standard for many open-source projects. It might seem like a lot of steps at first, but it's designed to keep the project clean and organized, and to make sure we can all work together without accidentally breaking things.

Here's the basic idea:

1.  You create a personal copy of our project on GitHub (called a **fork**).
2.  You download that copy to your computer (called **cloning**).
3.  You create a new **branch** for your changes. Think of a branch as a separate workspace where you can make edits without affecting the main codebase.
4.  You make your changes, and when you're done, you "push" them up to your fork on GitHub.
5.  You open a **Pull Request (PR)**, which is a formal way of asking us to review your changes and "pull" them into the main project.

This process ensures that all changes are reviewed before they become part of the main project, which helps us catch bugs and maintain quality.

## Your First-Time Setup (You only need to do this once)

### Step 1: Fork the Repository

A fork is your own personal copy of the repository on GitHub. You can make any changes you want to your fork without affecting the main project.

1.  Go to the main project repository page: [https://github.com/mmandernach13/UNL-UAV](https://github.com/mmandernach13/UNL-UAV)
2.  In the top-right corner of the page, click the **Fork** button.

You now have a copy of the repository in your own GitHub account!

### Step 2: Clone Your Fork

Now, you need to download your fork to your computer. This is called "cloning".

1.  On your fork's GitHub page, click the green **Code** button.
2.  Make sure `HTTPS` is selected, and copy the URL. It should look something like this: `https://github.com/YOUR_USERNAME/UNL-UAV.git`
3.  Open a terminal on your computer and run this command (replace `YOUR_USERNAME` with your actual GitHub username):

    ```bash
    git clone https://github.com/YOUR_USERNAME/UNL-UAV.git
    ```

4.  Now, navigate into the newly created directory:

    ```bash
    cd UNL-UAV
    ```

### Step 3: Connect Your Fork to the Main Repository

You need to tell your local repository about the main project so you can keep it updated with the latest changes.

1.  In your terminal, add a new "remote" (a pointer to another repository) called `upstream` that points to the main project:

    ```bash
    git remote add upstream https://github.com/mmandernach13/UNL-UAV.git
    ```

2.  You can check that it worked by running:

    ```bash
    git remote -v
    ```

    You should see your fork listed as `origin` and the main project listed as `upstream`.

## The Daily Workflow: Making a Change

Now that you're all set up, here's how you'll make changes.

### Step 1: Get the Latest Changes

Before you start working, you should always make sure your local copy is up-to-date with the main project.

1.  Make sure you are on your `main` branch:
    ```bash
    git checkout main
    ```
2.  "Fetch" the latest changes from the `upstream` (the main project) and merge them into your local `main` branch:
    ```bash
    git pull upstream main
    ```

### Step 2: Create a New Branch

It's very important to work on a new branch for every new feature or bug fix. This keeps your changes isolated and easy to manage.

1.  Create a new branch and switch to it. Give it a descriptive name, like `feature/add-new-sensor` or `bugfix/fix-takeoff-logic`.

    ```bash
    git checkout -b feature/your-feature-name
    ```

### Step 3: Make Your Code Changes

This is the fun part! Open the project in your favorite editor and make the changes you want to contribute.

### Step 4: Save Your Changes (Commit)

When you've made some progress, you need to save a snapshot of your changes. This is called a "commit".

1.  You can see which files you've changed by running:
    ```bash
    git status
    ```
2.  Add the files you want to save to the "staging area". If you want to add all your changed files, you can do:
    ```bash
    git add .
    ```
3.  Now, "commit" your changes with a clear and descriptive message:
    ```bash
    git commit -m "feat: Add support for the new XYZ sensor"
    ```
    *Good commit messages are important!* Start with a prefix like `feat:`, `fix:`, `docs:`, or `refactor:` to describe the type of change.

### Step 5: Push Your Branch to Your Fork

Now you need to upload your new branch and its commits to your fork on GitHub.

```bash
git push -u origin feature/your-feature-name
```
(You only need the `-u` the first time you push a new branch).

## Creating a Pull Request (Asking for a Review)

A Pull Request (PR) is how you tell the project maintainers that you have some changes you'd like to merge into the main project.

1.  Go to your fork's page on GitHub (`https://github.com/YOUR_USERNAME/UNL-UAV`).
2.  You should see a yellow banner that says "feature/your-feature-name had recent pushes". Click the **Compare & pull request** button.
3.  This will take you to the "Open a pull request" page.
4.  Make sure the "base repository" is `mmandernach13/UNL-UAV` and the "base" branch is `main`. The "head repository" should be your fork, and the "compare" branch should be your feature branch.
5.  Give your PR a clear title and a description of the changes you made. Explain *why* you made them and *how* you tested them.
6.  Click **Create pull request**.

That's it! You've successfully submitted a contribution. The project maintainers will be notified, and they will review your changes. They might ask you to make some adjustments. Once everyone is happy, your changes will be merged into the main project.

## After Your PR is Merged

Once your PR is approved and merged, you can safely delete your branch.

1.  Switch back to your `main` branch:
    ```bash
    git checkout main
    ```
2.  Update your local `main` branch with the newly merged changes from `upstream`:
    ```bash
    git pull upstream main
    ```
3a. Delete the local branch:
    ```bash
    git branch -d feature/your-feature-name
    ```
3b. You can also delete the remote branch from your fork on GitHub.

Thank you for contributing to our project!