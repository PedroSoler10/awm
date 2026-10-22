# Running the tasks

## Model and effort per task

| Step | Who / model | Effort | Why |
|---|---|---|---|
| 0: commit, push, container check, task files | this planning session (Opus 5.5) | as is | Needs the full context of this analysis. Small. |
| 1: remove files | **Sonnet 5.5** | **Low** | Mechanical: deletions, CMake list, README text. The table already holds every decision. |
| 2: shared library + brain refactor | **Opus 5.5** | **Medium** | The foundation every later task uses. Moving code without changing behaviour (ring-search order, measure map after a resize), and setting up the library, gtest and install in CMake. A subtle mistake here spreads to every later task. |
| 3: map_processing | **Sonnet 5.5** | **Medium** | Small, well-specified node and one pure function with tests. |
| 4: goal_points service | **Sonnet 5.5** | **High** | New algorithms (frontier 3x3 check, BFS clustering), srv redesign with message dependencies, and generalising the ring search without breaking brain. |
| 5: brainT modular | **Sonnet 5.5** | **High** | Three nodes interacting in the sim. Most of the time goes to runtime debugging (service timing, frontier goals that must grow the map). |
| 6: mode switch | **Opus 5.5** | **Medium** | Design work: Strategy interface, merging two brains while standalone must stay identical, conditional launch. |
| 7: review + docs | **Opus 5.5** | **Medium** | The review needs judgment on which findings are real. The docs must match the final code. |

If you want to use only Sonnet: run Tasks 2 and 6 at **High**, and read their diffs more carefully.

## What you do for each task (repeat 7 times, in order)

1. Make sure the previous step is committed and pushed (`git status -sb` shows a clean tree and no `ahead`), and that the dev container is running (VS Code open on the folder in the container, or `docker start` as in the Dev container section).
2. **Start a new session** in the Claude desktop app (Code tab) on the folder `/home/pedro-soler/shared_ros_ws/src/awm`, **without a worktree**. The container mounts `~/shared_ros_ws`, so a worktree somewhere else would not be in the catkin workspace. The tasks are sequential anyway.
3. **Pick the model and effort** from the table, in the session's model picker, before sending the prompt.
4. **Paste the prompt for that task** (below) and send it. Accept the permission prompts for `docker exec`, `git` and file edits (or use "accept edits" mode).
5. **When it finishes, check before starting the next task:**
   ```bash
   git log -2 --stat
   ```
   ```bash
   tail -n 30 docs/refactor/PROGRESS.md
   ```
   If the report says a check failed or was skipped, tell the same session to fix it (for example: "The smoke test was not run, run it and fix what fails"). Move on only when the task is committed, pushed, and its "Done when" holds.
6. Optional: archive the finished session.

If a session finds that its task is wrong or impossible as written, it stops and explains. Take that to an Opus planning session to adjust the task files.

## Exact prompts

Every prompt has the same shape; only the number, the title and one check line change.

**Task 1** (Sonnet 5.5, Low)
```
Do Task 1 of the awm refactor: "Remove files with no value".
Read docs/refactor/PLAN.md (context, target structure, "Dev container", "Rules for every session"), then docs/refactor/task-1.md, then docs/refactor/PROGRESS.md for the baseline and what earlier steps did.
Do only Task 1. Use scripts/container_build.sh and scripts/smoke_test.sh, which reuse the existing dev container: never rebuild the image or recreate the container. Build and run the sim smoke test as the rules say, run scripts/check_format.sh, fix README sections your change makes wrong, append your entry to docs/refactor/PROGRESS.md, make one commit on branch code-review and push it to origin.
If something in the task is wrong or impossible, stop and tell me instead of improvising a different design. Do not start the next task.
```

**Task 2** (Opus 5.5, Medium)
```
Do Task 2 of the awm refactor: "Shared library, package layout, brain on top of it".
Read docs/refactor/PLAN.md (context, target structure, "Dev container", "Rules for every session"), then docs/refactor/task-2.md, then docs/refactor/PROGRESS.md for the baseline and what earlier steps did.
Do only Task 2. Use scripts/container_build.sh and scripts/smoke_test.sh, which reuse the existing dev container: never rebuild the image or recreate the container. Build, run the unit tests and the sim smoke test as the rules say (brain must behave exactly as in the baseline), run scripts/check_format.sh, fix README sections your change makes wrong, append your entry to docs/refactor/PROGRESS.md, make one commit on branch code-review and push it to origin.
If something in the task is wrong or impossible, stop and tell me instead of improvising a different design. Do not start the next task.
```

**Task 3** (Sonnet 5.5, Medium)
```
Do Task 3 of the awm refactor: "map_processing -> clean /safe_map producer".
Read docs/refactor/PLAN.md (context, target structure, "Dev container", "Rules for every session"), then docs/refactor/task-3.md, then docs/refactor/PROGRESS.md for the baseline and what earlier steps did.
Do only Task 3. Use scripts/container_build.sh and scripts/smoke_test.sh, which reuse the existing dev container: never rebuild the image or recreate the container. Build, run the unit tests and the sim checks the task asks for, run scripts/check_format.sh, fix README sections your change makes wrong, append your entry to docs/refactor/PROGRESS.md, make one commit on branch code-review and push it to origin.
If something in the task is wrong or impossible, stop and tell me instead of improvising a different design. Do not start the next task.
```

**Task 4** (Sonnet 5.5, High)
```
Do Task 4 of the awm refactor: "points_identification -> /goal_points service".
Read docs/refactor/PLAN.md (context, target structure, "Dev container", "Rules for every session"), then docs/refactor/task-4.md, then docs/refactor/PROGRESS.md for the baseline and what earlier steps did.
Do only Task 4. Use scripts/container_build.sh and scripts/smoke_test.sh, which reuse the existing dev container: never rebuild the image or recreate the container. Build, run the unit tests and the rosservice checks in the sim that the task asks for, run scripts/check_format.sh, fix README sections your change makes wrong, append your entry to docs/refactor/PROGRESS.md, make one commit on branch code-review and push it to origin.
If something in the task is wrong or impossible, stop and tell me instead of improvising a different design. Do not start the next task.
```

**Task 5** (Sonnet 5.5, High)
```
Do Task 5 of the awm refactor: "brainT -> modular brain that uses the services".
Read docs/refactor/PLAN.md (context, target structure, "Dev container", "Rules for every session"), then docs/refactor/task-5.md, then docs/refactor/PROGRESS.md for the baseline and what earlier steps did.
Do only Task 5. Use scripts/container_build.sh and scripts/smoke_test.sh, which reuse the existing dev container: never rebuild the image or recreate the container. Build, run the unit tests and the sim smoke tests for both the standalone and the modular pipeline, run scripts/check_format.sh, fix README sections your change makes wrong, append your entry to docs/refactor/PROGRESS.md, make one commit on branch code-review and push it to origin.
If something in the task is wrong or impossible, stop and tell me instead of improvising a different design. Do not start the next task.
```

**Task 6** (Opus 5.5, Medium)
```
Do Task 6 of the awm refactor: "One brain, mode chosen by a parameter".
Read docs/refactor/PLAN.md (context, target structure, "Dev container", "Rules for every session"), then docs/refactor/task-6.md, then docs/refactor/PROGRESS.md for the baseline and what earlier steps did.
Do only Task 6. Use scripts/container_build.sh and scripts/smoke_test.sh, which reuse the existing dev container: never rebuild the image or recreate the container. Build, run the unit tests and the sim smoke test for mode:=standalone (must match the baseline) and mode:=modular, run scripts/check_format.sh, fix README sections your change makes wrong, append your entry to docs/refactor/PROGRESS.md, make one commit on branch code-review and push it to origin.
If something in the task is wrong or impossible, stop and tell me instead of improvising a different design. Do not start the next task.
```

**Task 7** (Opus 5.5, Medium)
```
Do Task 7 of the awm refactor: "Final review and documentation".
Read docs/refactor/PLAN.md (context, target structure, "Dev container", "Rules for every session"), then docs/refactor/task-7.md, then docs/refactor/PROGRESS.md for everything earlier steps did.
Do only Task 7. Use scripts/container_build.sh and scripts/smoke_test.sh, which reuse the existing dev container: never rebuild the image or recreate the container. Run /code-review high on main..code-review and fix the confirmed findings. Run scripts/check_format.sh with no exclusions, the unit tests, and the final sim smoke test of both modes. Update README and docs/CONFIGURATION.md. Report the PROGRESS.md summary and the final results to me, then delete docs/refactor/, make the final commit on branch code-review and push it to origin.
If something is wrong or impossible, stop and tell me instead of improvising.
```

Tasks 3 and 4 touch different nodes, but both edit CMakeLists.txt and the library, so keep them one after the other.
