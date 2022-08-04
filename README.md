DISCONTINUATION OF PROJECT.

This project will no longer be maintained by Intel.

Intel has ceased development and contributions including, but not limited to, maintenance, bug fixes, new releases, or updates, to this project. 

Intel no longer accepts patches to this project.

If you have an ongoing need to use this project, are interested in independently developing it, or would like to maintain patches for the open source software community, please create your own fork of this project. 
# corelibs-galileo

The contents of this repo is distributed through releases in Arduino IDE.    
`Tools > Board > Boards Manager > Intel i586 Boards by Intel`

If you wish to use the latest **untested** changes, follow these instructions.

1. Install the latest `Intel i586 Boards by Intel` from `Boards Manager`
2. Download the [latest snapshot](https://github.com/01org/corelibs-galileo/archive/master.zip)
   of this repo
3. Shut down the IDE
4. Go to Arduino15 directory
  * Windows: `C:\Users\<user>\AppData\Roaming\Arduino15`
  * OS X: `~/Library/Arduino15`
  * Linux: `~/.arduino15`
5. Go to `packages/Intel/hardware/i586/<version>/`
6. Delete the content of the directory from step 5, and replace it with the
   content of the "corelibs-edison-master" folder in the zip from step 2.

Future upgrades may fail since the internal contents were modified by hand. In
order to recover, shut down the IDE, delete the entire `Arduino15` directory,
then restart the IDE.

# Support & Issues

If you have found a bug, or you believe a new feature should be added, please
use the Github issue tracker (click "Issues" above) to provide details about
the bug or feature. If you need product support (e.g. have a question about /
are having problems with the Arduino IDE or the Arduino API), please direct
them to the [support forum](https://communities.intel.com/community/tech/galileo).

## Examples of things that should go in the Issue tracker

> "I noticed that your DoSomeThing library doesn't support all the same
> modes as the library from SomeOtherGuy: https://link-to-relevant-thing.com
> Can you add support for these modes?"

> "If I run example sketch X on a Galileo board, I get result Y. But if I
> run the same sketch on an Arduino UNO board, I get result Z. This looks like
> a bug to me."

## Examples of things that should go in the support forum

> "I'm having trouble downloading the Galileo boards package in the Arduino
> IDE Boards Manager"

> "How do I use this library?"

> "I can't get this example sketch to work. What am I doing wrong?"
