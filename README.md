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

#### Repository Checkout & Submodule Init/Update

Note that if you don't want to deal with version control, just download a snapshot of the branch you want as a zip file (GitHub `Download ZIP` link at top-right of files list in `Code` view -- be sure to first select the branch you want to download). Otherwise, read this section.

Make sure to initialize and update submodules (MAVLink headers). When you first clone the repo this will most likely be done automatically for you. If you're updating an existing local copy, you will most likely have to "init" and "update" the submodules manually. After checking out or downloading the repo, make sure the `lib/mavlink/include` folder exists.  Most GUI Git clients will have a way to update submodules, or you can use the command line.

Command-line example (run this in the local repo folder):
```shell
git submodule update --init --recursive
```

Here's a more complete example starting with fresh copy of the repo, submodule init, and then checking out the `next` branch:

```shell
git clone https://github.com/mpaperno/aq_flight_control.git
cd aq_flight_control
git submodule update --init --recursive
git checkout next
```

#### Compiling The Firmware

##### Building with CrossWorks for ARM:

Note: If you already own a full (not eval) license for CrossWorks for ARM, simply install version 2.3.5 from their archives page, open the autoquad.hzp file, and build one of the Release types.  Otherwise, proceed as below.

1. Install, start, and license (eval is OK) latest version of [CrossWorks for ARM](http://www.rowley.co.uk/arm/index.htm). 
  1. You may also need to install the `STM32 CPU Support Package` "legacy" version 2.28 (use the `Package Manager` found in `Tools` menu). You can always install it later if the build doesn't work w/out it (we're not actually using anything from this package).
2. Download and unpack/install CrossWorks for ARM 2.3.5 from http://www.rowley.co.uk/arm/releases.htm .  (You do not need to run or activate this version, we only need the libraries from here.)
3. Copy the `lib` folder from CW 2.3.5 install/unpack folder to the installation folder of the latest CW version you installed.  To avoid conflict with the existing `lib` folder, rename the copied one to `lib-2.3`.  If you are not going to use CW for anything else, you can just replace the existing `lib` folder and skip step 5, below.  You can now uninstall/delete the CW 2.3.5 installation.
4. Open autoquad.hzp file in CrossWorks (the one from step 1).
5. (Skip this step if you replaced the `lib` folder in step 3.) 
  1. In the Project Explorer window, expand the `Project 'autoquad'` and then the `Project Properties` trees.
  2. Double-click the `Additional Input Files` line.
  3. Replace each occurrence of `$(StudioDir)/lib` with the folder where you copied the CW 2.3.5 libraries to.  For example: `$(StudioDir)/lib-2.3`
  4. Click OK and you're done.
6. In Project Explorer window, select the build type from the dropdown menu at the top left.  The build type should match your AQ hardware (version, IMU type, etc). Do not select a "Debug" type build unless you intend to use hardware debugging.
7. Select `Build Solution` from the main `Build` menu, or press SHIFT-F7.  If all goes well, there should be a compiled firmware binary in a subfolder of the `build` directory.

##### Building with Makefile:

1. Download and unpack/install CrossWorks for ARM 2.3.5 from http://www.rowley.co.uk/arm/releases.htm . It is best to install it to a directory path with no spaces in the names at all. You do not need to run this version.
2. In a plain-text editor, create a new file in the root of this project named `Makefile.user` (it goes next to the existing `Makefile`).
3. Put the following on one line of the new file: `CC_PATH ?= [path to CW]` where "[path to CW]" is the root of your CrossWorks 2.3 installation folder (at minimum, the following folders are expected to be inside the CW install folder: `/gcc/arm-unknown-eabi/bin, lib, include`). Eg. `CC_PATH ?= /usr/share/crossworks_for_arm` or `CC_PATH ?= c:/devel/crossworks_for_arm`.  **On Windows** always use forward slashes in directory paths (see Notes for Windows users, below).
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
4. You can easily set up a local environment and specify build options using a batch or shell file. This is especially useful for Windows so you can specify a local `PATH` variable w/out having to change the system-wide PATH (and need to restart Windows). The order of the PATH entries also affects how Windows searches for commands (making it easy to, for example, override Windows' `mkidir` with GNU tools `mkdir`). Here is a basic example batch file to initiate a build (this also shows using environment variables to set all the build options):

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

Most distributions include an older version of 'make' (3.x). Version 4.x offers some improvements for parallel builds. Windows versiond are available from [ezwinports](http://sourceforge.net/projects/ezwinports/files/) (get a "w/out guile" version) or [Equation Solution](http://www.equation.com/servlet/equation.cmd?fa=make) (64 or 32 bit version, depending on your Windows type).

Note that all directory paths used by `make` should have forward slashes (`/`) instead of typical Windows backslashes (`\`).

