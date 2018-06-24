# Set some sane defaults for the dyno launch environment

##Documentation:
#  The colon command simply has its arguments evaluated and then succeeds.
#   It is the original shell comment notation (before '#' to end of line). For a long time, Bourne shell scripts had a colon as the first character.
#   The C Shell would read a script and use the first character to determine whether it was for the C Shell (a '#' hash) or the Bourne shell (a ':' colon).
#   Then the kernel got in on the act and added support for '#!/path/to/program' and the Bourne shell got '#' comments, and the colon convention went by the wayside.
#   But if you come across a script that starts with a colon (Like this one), now you will know why. ~ Jonathan Leffler
# Set some sane defaults for the dyno launch environment

##Documentation:
#  The colon command simply has its arguments evaluated and then succeeds.
#   It is the original shell comment notation (before '#' to end of line). For a long time, Bourne shell scripts had a colon as the first character.
#   The C Shell would read a script and use the first character to determine whether it was for the C Shell (a '#' hash) or the Bourne shell (a ':' colon).  #   Then the kernel got in on the act and added support for '#!/path/to/program' and the Bourne shell got '#' comments, and the colon convention went by the wayside.
#   But if you come across a script that starts with a colon (Like this one), now you will know why. ~ Jonathan Leffler

: ${DYNO_NAME:=Ann}
: ${DYNO_SIMULATION:=true}
: ${DYNO_BASE:=omnibot}
: ${DYNO_RAPP_PACKAGE_WHITELIST:=[rocon_apps, dyno_rapps]}
: ${DYNO_RAPP_PACKAGE_BLACKLIST:=[]}
: ${DYNO_INTERACTIONS_LIST:=[dyno_interactions/web, dyno_interactions/pairing]}
: ${DYNO_HUB_URI:=http://localhost:6380}
: ${DYNO_CONCERT_DISABLE_ZEROCONF:=false}
: ${DYNO_DEFAULT_RAPP:=dyno_rapps/waypoint_navigation}
: ${DYNO_PERFORMANCE:=false}
: ${DYNO_FAKE_ODOM:=false}
: ${JOY_SERIAL_PORT:=/dev/input/js0}
: ${ROS_MASTER_URI=http://localhost:11311}
: ${ROS_HOSTNAME=localhost}

# Exports
export DYNO_NAME
export DYNO_SIMULATION
export DYNO_BASE
export DYNO_RAPP_PACKAGE_WHITELIST
export DYNO_RAPP_PACKAGE_BLACKLIST
export DYNO_INTERACTIONS_LIST
export DYNO_HUB_URI
export DYNO_CONCERT_DISABLE_ZEROCONF
export DYNO_DEFAULT_RAPP
export DYNO_PERFORMANCE
export DYNO_FAKE_ODOM
export JOY_SERIAL_PORT
export ROS_MASTER_URI
export ROS_HOSTNAME
