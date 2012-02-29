This is the source code for the ANCL Heliocpter autopilot.

Directory structure:

src/ - Source files
src/tests/ - test files for isolating components of the autopilot
extern/ - External library headers
lib/ - Precompiled external libraries

============================

To import the source into QNX Momentics IDE:

1. File->New->QNX C++ Project
2. Project name = Autopilot
3. Uncheck "Use default location"
4. Set location to src directory
5. Uncheck "Generate default file"
6. Click Next
7. Select appropriate Build Variant (e.g., x86 debug)
8. Click Finish

============================

Generate Documentation

$ cd src
$ doxygen

the documentation will be placed in the doc directory.

============================

MAVLink Dependency

The official MAVLink source can be found at https://github.com/mavlink/mavlink

However, this autopilot is built against a custom version which is not always in sync with the official version.
Therefore, it is recommended that to build the autopilot you clone our custom version available from
https://github.com/ancl/mavlink.  Note, we are currently using the v10release branch.

Similar to QGroundControl (http://qgroundcontrol.org) this software expects MAVLink to be stored in the same directory as the autopilot source.
In other words, after cloning the autopilot and mavlink you should see both of the folders in a single directory listing.