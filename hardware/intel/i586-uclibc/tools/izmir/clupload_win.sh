#!/bin/sh

echo "starting download script"
echo "Args to shell:" $*

# ARG 1: Path to lsz executable.
# ARG 2: Elf File to download
# ARG 3: COM port to use.

#path contains \ need to change all to /
path_to_exe=$1
fixed_path=${path_to_exe//\\/\/}

#COM ports are not always setup to be addressed via COM for redirect.
#/dev/ttySx are present. Howwever, COMy -> /dev/ttySx where x = y - 1

com_port_arg=$3
com_port_id=${com_port_arg/COM/}
echo "COM PORT" $com_port_id
tty_port_id=/dev/ttyS$((com_port_id-1))
echo "Converted COM Port" $com_port_arg "to tty port" $tty_port_id

echo "Sending Command String to move to download if not already in download mode"
echo "~sketch downloadGalileo" > $tty_port_id

#Move the existing sketch on target.
echo "Deleting existing sketch on target"
"$fixed_path/lsz.exe" --escape -c "mv -f /sketch/sketch.elf /sketch/sketch.elf.old" <> $tty_port_id 1>&0
# Execute the target download command

#Download the file.
host_file_name=$2
"$fixed_path/lsz.exe" --escape --binary --overwrite $host_file_name <> $tty_port_id 1>&0

#mv the downloaded file to /sketch/sketch.elf 
target_download_name="${host_file_name##*/}" 
echo "Moving downloaded file to /sketch/sketch.elf on target"
"$fixed_path/lsz.exe" --escape -c "mv $target_download_name /sketch/sketch.elf; chmod +x /sketch/sketch.elf" <> $tty_port_id 1>&0
