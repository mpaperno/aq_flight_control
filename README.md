# AutoQuad Flight Control Firmware

This is a fork/continuation of [Bill Nesbitt's AutoQuad firmware](https://github.com/bn999/autoquad) for multicopter flight control.

* [AutoQuad Project Site](http://autoquad.org/)
* [AutoQuad Documetation](http://autoquad.org/wiki/wiki/)
* [AutoQuad Forums](http://forum.autoquad.org/)
* [AutoQuad Downloads Page](http://autoquad.org/software-downloads/)
* [AutoQuad Public FTP: ftp://ftp.autoquad.org/3/334693_22529/](ftp://ftp.autoquad.org/3/334693_22529/)

### USE AT YOUR OWN RISK!!! ABSOLUTELY NO RESPONSOBILITY IS ASSUMED BY THE AUTHORS OR PUBLISHERS OF THIS FIRMWARE. SEE LICENSE FILE FOR FULL TERMS.

#### Repository Structure:

  * `master` branch is current stable version.
  * `next` branch is to integrate all proposed changes for holistic testing before being committed to `master`.
  * Numbered branches (eg. `6.8`) are for older versions.
  * All other branches are "feature" branches used for testing individual changes before integration into `next`.

Pull requests should typically be submitted against the `next` branch, unless it is an important fix for something which affects `master` as well, or some other similarly good reason.

#### Binary Distributions

Pre-compiled firmware versions can be found at the [AutoQuad public FTP site: ftp://ftp.autoquad.org/3/334693_22529/aq-firmware/forks/mpaperno/aq_flight_control/](ftp://ftp.autoquad.org/3/334693_22529/aq-firmware/forks/mpaperno/aq_flight_control/).  The structure is organized by repository branch and then hardware version.

#### Getting The Source Code

If you already have a Git client installed (or are willing to learn), the easiest method is to clone this repository to your local machine. If you do not want to deal with Git, you may also download zip archives of the necessary projects.

##### Repository Checkout & Submodule Init/Update

This repository contains [Git submodules](http://blogs.atlassian.com/2011/12/git-submodules/) (for MAVLink headers) which must be initialized and updated. This is a simple process but you will get compile errors if you don't do it.

If you use a GUI Git client ([TortoiseGit](https://tortoisegit.org/docs/tortoisegit/tgit-dug-submodules.html), [SourceTree](https://blog.sourcetreeapp.com/2012/02/01/using-submodules-and-subrepositories/), etc) then look for options such as _recursive_ during cloning and commands like "Update Submodules". This is usually an option when (or after) you do a clone/checkout command. Refer to the program's help if necessary. After checking out the code, make sure the `lib/mavlink/include` folder exists.

It is also very easy to use the command line for clone, update, and checkout.

Here's a complete example starting with fresh copy of the repo, then checking out the `next` branch, and the submodule init:

```shell
git clone https://github.com/mpaperno/aq_flight_control.git
cd aq_flight_control
git checkout next
git submodule update --init
```

If you already have a clone of the repo and you only want to do a pull of the latest changes, run something like this (example uses the `master` branch and assumes "origin" for the remote name of this GitHub repo, which is a typical default):

```shell
git checkout master
git pull --recurse-submodules origin master
git submodule update --init
```

As you may have gathered by now, the point is to run `git submodule update` after cloning or updating the code from this repository.  The `--init` option is only necessary the first time, but it doesn't hurt to include it.

##### Download Code as Zip Archives

Unfortunately GitHub makes this a bit more complicated than it should be. To download a snapshot of the current code on any branch:

1. In the default GitHub _Code_ tab view, find the _Branch_ menu and select the branch you want to download.
2. To the right of the _Branch_ menu, click the _Download ZIP_ link and save the file (it will be named something like "aq\_flight\_control\_master.zip")
3. While still in the _Code_ view, click on the `lib` folder.
4. To the right of the `mavlink` folder there is an ID like "67a140b" or similar (7 hex digits).  Click on that. It will take you to a different code repository (AutoQuad/mavlink_headers) and to a specific commit in a specific branch (this is important).
5. Now click the _Download Zip_ link on this new page. Save the file (it will be named "mavlink\_headers-" with a long string of numbers at the end).
6. Unzip the "aq\_flight\_control\_master.zip" file into wherever you want to keep the firmware source code (preferably a directory path w/out spaces).
7. Unzip the "mavlink\_headers-xxxxx.zip" file into the `lib/mavlink` folder of the firmware source code tree.  So the final result should be a `lib/mavlink/include` folder with 2 subfolders and some .h files inside.


#### Compiling The Firmware

##### Building with CrossWorks for ARM:

Note: If you already own a full (not eval) license for CrossWorks for ARM, simply install version 2.3.5 from their archives page, open the autoquad.hzp file, and build one of the Release types.  Otherwise, proceed as below.

1. Install, start, and license (eval is OK) latest version of [CrossWorks for ARM](http://www.rowley.co.uk/arm/index.htm). 
  1. You may also need to install the `STM32 CPU Support Package` "legacy" version 2.28 (use the `Package Manager` found in `Tools` menu). You can always install it later if the build doesn't work w/out it (we're not actually using anything from this package but CW may complain w/out it).
2. Download and unpack/install CrossWorks for ARM 2.3.5 from http://www.rowley.co.uk/arm/releases.htm .  (You do not need to run or activate this version, we only need the libraries from here.)
3. Copy the `lib` folder from CW 2.3.5 install/unpack folder to the installation folder of the latest CW version you installed in step 1.  To avoid conflict with the existing `lib` folder, rename the copied one to `lib-2.3`.  If you are not going to use CW for anything else, you can just replace the existing `lib` folder and skip step 5, below.  You can now completely remove the CW 2.3.5 installation/files.
4. Open autoquad.hzp file in CrossWorks (the one from step 1).
5. (Skip this step if you replaced the `lib` folder in step 3.) 
  1. In the Project Explorer window, expand the `Project 'autoquad'` and then the `Project Properties` trees.
  2. Double-click the `Additional Input Files` line.
  3. Replace each occurrence of `$(StudioDir)/lib` with the folder where you copied the CW 2.3.5 libraries to.  For example: `$(StudioDir)/lib-2.3`
  4. Click OK and you're done.
6. In Project Explorer window, select the build type from the dropdown menu at the top left.  The build type should match your AQ hardware (version, IMU type, etc). Be sure to select a "Release" type build (not "Debug").
7. Select `Build Solution` from the main `Build` menu, or press `SHIFT-F7`.  If all goes well, there should be a compiled firmware binary in a subfolder of the `build` directory.

##### Building with Makefile:

1. Download and unpack/install CrossWorks for ARM 2.3.5 from http://www.rowley.co.uk/arm/releases.htm . It is best to install it to a directory path with no spaces in the names at all. You do not need to run this version, we just need the build toolchain (compiler and libraries).
2. In a plain-text editor, create a new file in the root of this project named `Makefile.user` (it goes next to the existing `Makefile`).
3. Enter the following on a line of the new file: `CC_PATH ?= [path to CW]` where "[path to CW]" is the root of your CrossWorks 2.3 installation folder (at minimum, the following folders are expected to be inside the CW install folder: `/gcc/arm-unknown-eabi/bin, lib, include`). Eg. `CC_PATH ?= /usr/share/crossworks_for_arm` or `CC_PATH ?= c:/devel/crossworks_for_arm`.  **On Windows** always use forward slashes in directory paths (see Notes for Windows users, below).
4. **Windows only:** in `Makefile.user`, also specify a path to the `mkdir` utility, like this:<br>
    `EXE_MKDIR = c:/GnuWin/bin/mkdir` where the "c:/GnuWin/bin/" part would be wherever you have installed GNU tools (see Notes for Windows users, below). _You can avoid this step if your GNU tools are on the `PATH` before the Windows system folders (see example batch file, below)._
5. Open a terminal/command prompt and navigate (`cd`) to the root of the project (where the Makefile lives). 
6. Type the command `make all` and see what happens.  With no other arguments, this should build a default firmware version for AQ6 revision 1 with DIMU. The binary should appear in a new `build/Release` folder.
7. To change the AQ hardware version, pass `BOARD_VER` and `BOARD_REV` arguments to `make`.  Eg. `make BOARD_VER=8 BOARD_REV=6` to build for M4 rev 6 (M4 v2). Read the `Makefile` for full list of versions and revisions available.

###### Tips for using Makefile

1. Read the Makefile comments for more options, full list of build targets, and other information.
2. Use `-jX` for faster (parallel) compilation, where "X" equals the number of CPUs/cores/threads on your computer.  If you have `make` version 4+, also add the `-O` option for better progress output during compilation. Eg. `make -j8 -O` for a quad-core CPU.
3. The `Makefile.user` file is the right place to specify build options you typically want, then you can avoid entering them on the command-line each time, or editing the main `Makefile`. You can also set any variable in the environment and `make` will use it instead of the default in the `Makefile`. Command-line options always override all other variables.
4. All directory paths are relative to the location of the `Makefile`. You can use relative or absolute paths for most options.
4. You can easily set up a local environment and specify build options using a batch or shell file. This is especially useful for Windows so you can specify a local `PATH` variable w/out having to change the system-wide `PATH` (and need to restart Windows). The order of the `PATH` entries also affects how Windows searches for commands (making it easy to, for example, override Windows' `mkidir` with GNU tools `mkdir`). Here is a basic example batch file to initiate a build (this also shows using environment variables to set all the build options):

```batchfile
@echo off
set PATH=c:\devel\GnuWin\bin;C:\Windows\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem
set CC_PATH=c:/devel/crossworks_for_arm_2.3
set BUILD_TYPE=Release-M4.r6
set BOARD_VER=8
set BOARD_REV=6
make bin 
```

###### Notes for Windows Users:

You will need some GNU (Unix/Linux) tools installed and in your [`PATH`](http://www.howtogeek.com/118594/how-to-edit-your-system-path-for-easy-command-line-access/). Make sure to install them on a path with no spaces (eg. `c:/GnuWin/`) There are several good sources for these, including [GnuWin32 CoreUtils](http://gnuwin32.sourceforge.net/packages/coreutils.htm) and [ezwinports](http://sourceforge.net/projects/ezwinports/files/). The following utilities are required: 

`sh, make, gawk, mv, echo, rm, mkdir, expr, zip`. 

Most distributions include an older version of 'make' (3.x). Version 4.x offers some improvements for parallel builds. Windows versions are available from [ezwinports](http://sourceforge.net/projects/ezwinports/files/) (get a "w/out guile" version) or [Equation Solution](http://www.equation.com/servlet/equation.cmd?fa=make) (64 or 32 bit version, depending on your Windows type).

Note that all directory paths used by `make` should have forward slashes (`/`) instead of typical Windows backslashes (`\`).

