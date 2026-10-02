#!/bin/sh
# Show an image on the phone's screen:  phone/show.sh picture.jpg
# (needs the phone's sshd and a web server in ~/ellama; see phone/README notes)
K="-i $HOME/.ssh/ellama_phone_ed25519 -o IdentitiesOnly=yes -o BatchMode=yes"
scp -q $K -P 8022 "$1" termux@192.168.0.54:ellama/show.jpg
