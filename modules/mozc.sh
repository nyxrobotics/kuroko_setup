#!/bin/bash

## >>>Install Mozc(Japanese input)
sudo apt install gnome-tweak-tool -y ibus-mozc emacs-mozc language-pack-ja
killall ibus-daemon
ibus-daemon -d -x &

## US Keyboard
gsettings set org.gnome.desktop.input-sources sources "[('xkb', 'us'), ('ibus', 'mozc-jp')]"

## Japanese Keyboard
# gsettings set org.gnome.desktop.input-sources sources "[('xkb', 'jp+OADG109A'), ('ibus', 'mozc-jp')]"

## Set the switch- shortcut key combination to Ctrl+space
gsettings set org.gnome.desktop.wm.keybindings switch-input-source "['<Primary>space']"
gsettings set org.gnome.desktop.wm.keybindings switch-input-source-backward "['<Shift><Primary>space']"


## Create the directory if it does not exist
mkdir -p "$HOME/.config/mozc"

## Write the specified content to the file
# cat << EOF > "$HOME/.config/mozc/ibus_config.textproto"
# engines {
#   name : "mozc-jp"
#   longname : "Mozc"
#   layout : "default"
#   composition_mode: HIRAGANA
# }
# EOF
# ibus write-cache; ibus restart

