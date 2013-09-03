#!/bin/bash
cat /dev/input/js0 | ssh root@192.168.178.28 'cat > /home/root/remote_dev/js0'


