# Contributing

## Branching Strategy

This repository follows the [GitFlow Workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow).  
If you're not familiar with it, please take a moment to review the basics.

## Think Before You Fork

Before creating a fork, please open an issue on the GitHub page.  
That gives maintainers an opportunity to:
- Confirm the bug or feature isn’t already in development
- Share helpful context or existing work
- Ensure alignment on goals and design

Avoid forking for changes that alter existing behavior without prior discussion.

## Fixing a Bug in Production Code

To fix a bug, create a branch from `master`.  
This branch should be named using the format: `hotfix/x.y.z-description`

Where `x.y.z` follows [Semantic Versioning](https://semver.org/).

Once complete, submit a pull request (PR) to `master`.  
A maintainer will review and merge the change.

## Adding a Feature

To add a new feature, branch from `devel`.

Once your work is complete and tested, open a PR to `devel`.  
If accepted, your contribution will be included in the next stable release from `master`.

## Additional Guidelines

- Include or update documentation (Doxygen/README/etc.) as appropriate
- Keep commits focused and meaningful (squash if needed before PR)
- Ensure `colcon build` passes with no errors or warnings

---

Thank you for helping make this project better! 🛰️
