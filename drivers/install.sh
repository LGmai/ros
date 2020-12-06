#!/bin/bash
RED='\033[0;31m'
NC='\033[0m' # No Color

# check if sudo is used
if [ "$(id -u)" != 0 ]; then
  echo 'Sorry, you need to run this script with sudo'
  exit 1
fi

cd "$(dirname "$0")"

if [ ! -d "$(uname -r)" ] || [ "$1" == "-c" ] || [ "$1" == "--compile" ]; then
  echo "Checking if build essential is installed"
  if [ $(dpkg-query -W -f='${Status}' make 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
    echo -e "${RED}Install build essential first - needed to make modules!${NC}"
    exit $?
  fi
  if [ -d "/lib/modules/$(uname -r)/extra/" ]; then
    rm -r "/lib/modules/$(uname -r)/extra/"
  fi
  make compile install
  if [ $? -ne 0 ]; then # Test exit status of make.
    echo -e "${RED}make error ${$?}!${NC}"
    exit $?
  fi
fi

if [ "$1" == "--clean" ]; then
  echo "Cleaning up"
  make clean
  if [ -d "/lib/modules/$(uname -r)/extra/" ]; then
    rm -r "/lib/modules/$(uname -r)/extra/"
  fi
fi

if [ -d "$(uname -r)" ]; then
  cp -rv "$(uname -r)/drivers" "/lib/modules/$(uname -r)/kernel/"
fi

depmod -a

