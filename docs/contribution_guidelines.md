# Contribution Guidelines

## Issues and Merge Requests

- Create issues for feature request or bug report. Issues are a nice way to
  describe in enough detail 
  - requirements of a feature
  - steps to reproduce a bug
  - effects of a bug
  - and more
  They allow all the participant to communicate in an asynchronous way and track
  the updates.
- Outside of initial development phase, please create merge requests for
  changes. Refrain from pushing directly to one of the stable branches like
  `master` or `develop`.
- Name the branch based on the changes being made.
  - `feature/<SPECIFICS>` for a branch which implements a new feature. For
    example, `feature/odom_monitor`, `feature/32_bit_crc`.
  - `fix/<SPECIFICS>` for a branch that addresses a bug. For example,
    `fix/laser_monitor_timeout`, `fix/crc_byte_overflow`.
  - `refactor/<SPECIFICS>` for a branch which refactors existing features but
    does not address any specific bug. For example, `refactor/crc`,
    `refactor/status_calc`.
- Add `labels` in merge requests and issues to help quickly sort and organise
  them.
- For new features, add corresponding tests.

## Commits

- Make individual commits for individual changes
- Do not make a single commit which addresses multiple issues or multiple logical changes
- For example, if you made changes to `A.cpp` and `B.h` but they are not related
  to each other directly, then make two separate commits. This includes changes
  that are very very small like fixing a single typo. *Make. Separate. Commits.*
- Add identifiers at the beginning of commit message to help quickly sort,
  organise and search through commits later. For example, if a function is added
  in class `Apple` then the commit message could look like
  ```
  [Apple] Added function getSize
  ```
  Commit that affect multiple files/classes/modules should list all of them. For
  example, if a change is made in class `Apple` and a directly related change
  had to be made in `FruitFactory` and tests related to them then the commit
  message could look like
  ```
  [Apple][FruitFactory][tests] Added additional param for constructor
  ```
  Obviously, the exact wording of the message is left up to the developer.
- Add reference to issue when a commit directly addresses an issue. For example
  ```
  [Apple] Fixed red shade of color. See #31
  ```
