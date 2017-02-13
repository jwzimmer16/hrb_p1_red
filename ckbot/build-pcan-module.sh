#!/bin/bash
if [ "$UID" != "0" ]; then
  echo Must run this as super-user. Use 'sudo'
  exit 1
fi
if [ -e $1/Makefile ]; then
  echo Using $1 as driver directory
else
  #echo Driver path \'$1\' not found
  echo 'Usage build-pcan-module <driver-directory-path>'
  exit 2
fi
echo Your current kernel is `uname -r`
echo Please select a kernel
pushd /boot
select ker in vmlinuz-*; do
  break
done
popd
export hdr=${ker/vmlinuz-/linux-headers-}
if [ -L /usr/src/linux ]; then
  echo Found `ls -l /usr/src/linux`
else
  echo Your /usr/src/linux is not a symbolic link
  echo if it does not exist, use: sudo ln -s /usr/src/linux foo 
  echo   then re-run this script. Bailing out! 
  exit 3
fi
if [ -e /usr/src/$hdr ]; then
  echo Found $hdr headers
else
  echo Did not find /usr/src/$hdr -- do you have linux-headers installed for $ker?
  exit 4
fi
echo Replacing symbolic link /usr/src/linux '->' /usr/src/$hdr
rm /usr/src/linux
ln -s /usr/src/$hdr /usr/src/linux

echo '>>>>>>>' Building driver '<<<<<<<<'
pushd $1
make clean && make NET=NO PCC=NO && make install
popd

/sbin/modprobe pcan