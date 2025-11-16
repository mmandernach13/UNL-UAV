# A Repo Owner's Guide to Managing Contributions

This guide is for you, the repository owner. You've been working on the `main` branch, and now that you're expecting contributions, it's time to adopt a workflow that will keep your project stable and make it easy to manage changes from others.

The key principle is this: **The `main` branch should always be stable and working.**

To achieve this, you'll start using the same process you ask your contributors to use. It might feel like a few extra steps, but it will save you massive headaches down the road.

---

## Your New Workflow: Lead by Example

Even as the owner, you should stop committing directly to `main`. Instead, you'll make your changes in a separate **branch** and merge them through a **Pull Request (PR)**, just like everyone else.

**Why?**
*   **It protects `main`:** You can experiment freely in a branch without any risk of breaking the main codebase.
*   **It creates a record:** It documents your thought process and allows for self-review before merging.
*   **It's consistent:** It sets a clear and consistent example for all contributors.

### How to Make Changes (The Owner's Workflow)

1.  **Start from `main`:** Before you do anything, make sure you're on the `main` branch and it's up to date.
    ```bash
    git checkout main
    git pull origin main
    ```
    *(Here, `origin` is your main repository on GitHub.)*

2.  **Create a new branch:** Just like a contributor, create a descriptively named branch for your work.
    ```bash
    git checkout -b feature/my-awesome-new-idea
    ```

3.  **Make your changes:** Edit, add, and delete files as needed.

4.  **Commit your work:** Save your changes with a clear commit message.
    ```bash
    git add .
    git commit -m "feat: Implement the core logic for my awesome new idea"
    ```

5.  **Push your branch:** Send the branch up to your repository on GitHub.
    ```bash
    git push -u origin feature/my-awesome-new-idea
    ```

6.  **Open a Pull Request:**
    *   Go to your repository on GitHub.
    *   You'll see a prompt to create a PR from your new branch. Click it.
    *   Even though you're the owner, write a brief description. This is a great habit for keeping a historical record of *why* changes were made.
    *   Click "Create pull request".

7.  **Merge your own PR:** Since you're the owner, you can review and merge your own PR. This might feel silly, but it's the correct process. Click the **"Merge pull request"** button.
    *   **Pro-Tip:** Use the **"Squash and merge"** option if available. This combines all of your branch's commits into a single, clean commit on the `main` branch. It keeps your project history tidy.

---

## How to Manage Contributions from Others

This is the most important part of your role as an owner. When a contributor opens a Pull Request, you are the gatekeeper for the `main` branch.

### Step 1: Review the Pull Request

1.  Go to the **"Pull requests"** tab in your GitHub repository.
2.  Click on the PR you want to review.
3.  You'll see a few tabs. The most important one is **"Files changed"**.
4.  In this tab, you can see every line of code that has been added, removed, or changed.
5.  You can leave comments on specific lines by clicking the `+` icon that appears when you hover over them. Use this to ask questions or suggest improvements.

### Step 2: Request Changes or Approve

*   **If the PR needs work:**
    *   Leave your comments on the code.
    *   When you're done, go back to the "Conversation" tab and click **"Review changes"**.
    *   Select the **"Request changes"** option and submit your review. This will block the PR from being merged until the contributor has addressed your feedback.

*   **If the PR is good to go:**
    *   Click **"Review changes"** and select **"Approve"**.

### Step 3: Merge the Pull Request

Once a PR is approved (either by you or another team member), you can merge it.

1.  Go to the PR page.
2.  Click the **"Merge pull request"** button.
3.  Again, it's highly recommended to select **"Squash and merge"** and edit the commit message to be clear and concise.
4.  Confirm the merge.

The contributor's changes are now officially part of your `main` branch!

---

## Handling Merge Conflicts

**What is a merge conflict?** A merge conflict happens when two people change the same lines in the same file. Git doesn't know which version is correct, so it asks a human to decide.

When a contributor's PR has a conflict with `main`, GitHub will detect it and prevent you from merging.

**Who fixes it?** The person who opened the PR is responsible for fixing their conflicts.

**How do you guide them?**
You can leave a comment on their PR with these instructions:

> "Hey, this PR has a merge conflict with the `main` branch. To fix it, you'll need to update your branch with the latest changes from `main`. Please run the following commands in your branch, fix the conflicts, and then push the changes back up to this PR:
>
> ```bash
> git checkout your-branch-name
> git pull origin main
> ```
>
> After you run the `pull` command, Git will show you the files with conflicts. Open those files, manually edit them to resolve the differences, and then commit and push the fix."

---

## Keeping Forks Updated

Contributors are responsible for keeping their own forks and branches up to date. The `CONTRIBUTING.md` guide you created gives them the instructions they need to do this (`git pull upstream main`). You don't need to worry about this part.

By following this workflow, you ensure your project remains a stable, high-quality codebase that is easy for others to contribute to. Welcome to the world of collaborative development!
