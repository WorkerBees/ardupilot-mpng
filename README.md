# ardupilot-mpng fork for arduplane

This branch will not longer maintained, please use the newest version.
If you think this branch should be maintained regardless it's outdated version,
please create an issue with the reasons.

All credit for the MPNG HAL goes to SirAlex

This fork provides the latest stable version of ArduPlane with SirAlex's HAL patches to support additional boards.

## Please note
* The ArduCopter code in this fork is defunct (use SirAlex's code for that)
* I try to keep the code as "vanilla" to the official ArduPlane as possible,
  which means that i will revert changes made by SirAlex which also affect ArduPlane but are not related to HAL
* I use git rebase to get SirAlex's changes to my fork

## Difference between ArduCopter code and ArduPlane code

While both code repositories provide both directories and code for each others project,
the code it-self differs because the time between both releases differs.

Additional to that ArduCopter or ArduPlane code could contain additional changes which happened after
the separation of the code-lines but before the release itself.

Thats the main reason the ArduCopter code is defunct in this fork.

Sample:
  ArduCopter 3.0.1 code also contains ArduPlane code but the code is older than the ArduPlane code
  released with ArduPlane 2.74b.

## How to compile and configure MegaPirateNG
Follow instructions at: http://docs.megapirateng.com