#!/bin/sh

FW_PATH="/sys/class/firmware/vmcu"

die() {
	echo "$1"
	exit 1
}

print_usage() {
    echo "Usage: ${1} [OPTIONS] FIRMWARE"
    echo
    echo "Args:"
    echo " -h, --help    This message"
    echo
    echo "Returns 0 on success."
}

die () {
	echo $1
	exit 1
}

while [ $# -gt 0 ]; do
    case $1 in
    -h|--help)
        print_usage
        exit 1
        ;;
    *)
        firmware="$1"
        shift # past value
        ;;
  esac
done

[ "x$firmware" != "x" ] || die "Missing mandatory argument FIRMWARE"

echo 1 > "${FW_PATH}/loading" || die "Failed preparing firmware load"
cat "$firmware" > "${FW_PATH}/data" || die "Failed firmware load"
echo 0 > "${FW_PATH}/loading" || die "Failed initializing firmware load"

status="unknown"
while [ "$status" != "idle" ]; do
	sleep 1
	status="$(cat ${FW_PATH}/status)"
	[ $? -eq 0 ] || die "Failed reading status"
done

error="$(cat ${FW_PATH}/error)"
[ $? -eq 0 ] || die "Failed reading error"

[ "x$error" = "x" ] || die "Firmware update returned error: $error"

echo "Update complete!"
echo "Shutdown to finalize."
exit 0
 