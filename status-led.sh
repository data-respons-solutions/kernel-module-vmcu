#!/bin/sh

if [ $# -lt 1 -o "$1" = "-h" -o "$1" = "--help" ]; then
	echo "Usage: status-led [red|green|yellow|off] [blink]"
	exit 1
fi

die() {
	echo "$1"
	exit 1
}

[ "$1" = "red" -o "$1" = "green" -o "$1" = "yellow" -o "$1" = "off" ] || die "Invalid argument"
[ "$#" -eq 2 -a "$2" != "blink" ] && die "Invalid argument"

# Set brightness to 0 in order to disable blink
echo 0 >/sys/class/leds/vmcu\:\:led0/brightness || die "Failed setting brightness"

case $1 in
	red)
		echo 1 >/sys/class/leds/vmcu\:\:led0/brightness || die "Failed setting brightness"
		echo 1 0 >/sys/class/leds/vmcu\:\:led0/multi_intensity || die "Failed setting red"
		;;
		
	green)
		echo 1 >/sys/class/leds/vmcu\:\:led0/brightness|| die "Failed setting brightness"
		echo 0 1 >/sys/class/leds/vmcu\:\:led0/multi_intensity || die "Failed setting green"
		;;

	yellow)
		echo 1 >/sys/class/leds/vmcu\:\:led0/brightness || die "Failed setting brightness"
		echo 1 1 >/sys/class/leds/vmcu\:\:led0/multi_intensity || die "Failed setting yellow"
		;;
		
	off)
		;;	
esac

if [ $# -eq 2 ] && [ $2 = "blink" ]; then
	echo timer > /sys/class/leds/vmcu\:\:led0/trigger || die "Failed setting blink"
fi

exit 0
